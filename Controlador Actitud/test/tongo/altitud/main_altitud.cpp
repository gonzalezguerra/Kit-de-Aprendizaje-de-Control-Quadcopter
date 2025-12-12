#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <FS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <math.h> 

// Módulos
#include "sensores.h"
#include "MadgwickAHRS.h"
#include "LQRController.h"
#include "AltitudePID.h"
#include "Ultrasonic.h"

// ==========================================================
//                         CONSTANTES GLOBALES
// ==========================================================

// --- Frecuencias de Lazo ---
const int ATTITUDE_LOOP_HZ = 150;
const TickType_t xFrequencyAttitude = pdMS_TO_TICKS(1000 / ATTITUDE_LOOP_HZ);

const int ALTITUDE_LOOP_HZ = 40;
const TickType_t xFrequencyAltitude = pdMS_TO_TICKS(1000 / ALTITUDE_LOOP_HZ);

const int TELEMETRY_LOOP_HZ = 10; // Frecuencia de la telemetria. 5-20 Hz
const uint32_t TELEMETRY_INTERVAL_MS = 1000 / TELEMETRY_LOOP_HZ;

// Heartbeat variables (ahora solo como marcador, no como failsafe activo)
volatile uint32_t g_last_heartbeat_ms = 0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 200; // Mantenido, pero no usado para failsafe

// --- Controladores y Filtros ---
MadgwickAHRS* ahrs;
LQRController* attitudeController;
AltitudePID* altController;

// --- Pines y Hardware ---
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int TRIG_PIN = 25;
const int ECHO_PIN = 26;

static const uint8_t PWM_PINS[4] = {32, 27, 4, 2}; // M1, M2, M3, M4
static const uint8_t LEDC_CH[4] = {0, 1, 2, 3}; // Canales LEDC para PWM
static const uint8_t LEDC_RES_BITS = 16;
static const uint32_t SERVO_FREQ_HZ = 50;      // Frecuencia PWM para ESCs
static const uint16_t PULSE_MIN_US = 1100;
static const uint16_t PULSE_MAX_US = 2100;

// Objetos de Sensores (Globales)
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// --- Calibración ---
float g_b[3] = {-0.01f, -0.06f, -0.02f};
float a_A[3][3] = {
    {1.002644, -0.004764, -0.004747 },
    { 0.050504, 0.985736, 0.003380 },
    { -0.004747, -0.002533, 0.979294 }
};
float a_b[3] = {0.268676, 0.140280, 0.152916};
float m_A[3][3] = {
    {1.114758, 0.013255, 0.007983},
    {0.013255, 1.048723, -0.005001},
    {0.007983, -0.005001, 1.027957}
};
float m_b[3] = {8.096708, -6.303482, -7.222026};

// GANANCIAS LQR
Matrix3x3 K_i_gains = {{{0.1f, 0.0f, 0.0f}, 
                        {0.0f, 0.1f, 0.0f}, 
                        {0.0f, 0.0f, 0.00f}}};
Matrix3x3 K_q_gains = {{{3.5f, 0.0f, 0.0f}, 
                        {0.0f, 3.5f, 0.0f}, 
                        {0.0f, 0.0f, 0.0f}}};
Matrix3x3 K_w_gains = {{{0.3f, 0.0f, 0.0f}, 
                        {0.0f, 0.3f, 0.0f}, 
                        {0.0f, 0.0f, 0.0f}}};

// ==========================================================
//               ESTADO COMPARTIDO (CON MUTEX)
// ==========================================================

SemaphoreHandle_t xStateMutex;

struct SharedControlState {
    float roll_deg, pitch_deg, yaw_deg; 
    Vector3f torques = {0.0f, 0.0f, 0.0f}; 
    float altitude_cm = 0.0f;
    float throttle_base = 0.0f;
} sharedState;

// Flags de Estado y Comandos (accedidas por red y tareas)
#define FLAG_ARMED (1 << 0) 
volatile int16_t g_setpoint_cm = 50; 
volatile uint8_t g_flags = 0; 
const float THROTTLE_HOVER = 0.4f;

// ==========================================================
//               FUNCIONES AUXILIARES (PWM, MATH)
// ==========================================================

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int8_t clamp_i8_deg(float d){
    if (d > 127) return 127;
    if (d < -128) return -128;
    return (int8_t)lrintf(d);
}

inline uint32_t periodUs() { return 1000000UL / SERVO_FREQ_HZ; }
inline uint32_t maxDuty() { return (1UL << LEDC_RES_BITS) - 1; }

uint32_t pulseToDuty(uint32_t pulse_us) {
    return (uint64_t)pulse_us * maxDuty() / periodUs();
}

void writePulseUs(uint8_t motor_index, uint16_t us) {
    if (us < PULSE_MIN_US) us = PULSE_MIN_US;
    if (us > PULSE_MAX_US) us = PULSE_MAX_US;
    ledcWrite(LEDC_CH[motor_index], pulseToDuty(us));
}

uint16_t floatToPulseUs(float value) {
        return PULSE_MIN_US + (uint16_t)(value * (PULSE_MAX_US - PULSE_MIN_US));
}

// ==========================================================
//    TAREA 1: CONTROL DE ACTITUD (CORE 1 - ALTA PRIORIDAD)
// ==========================================================
void Task_AttitudeControl(void *pvParameters) {
    Serial.println("Tarea de Actitud iniciada en Core 1.");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float dt = 1.0f / ATTITUDE_LOOP_HZ;

    if ((g_flags & FLAG_ARMED) && (millis() - g_last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS)) {
        Serial.println("[FAILSAFE] Heartbeat perdido. DESARMANDO MOTORES.");
        g_flags &= ~FLAG_ARMED;
    }       
    
    for (;;) {
        // ALTITUD
        Vector3f torques_local = {0.0f, 0.0f, 0.0f};

        float throttle_base_local;

        if (xSemaphoreTake(xStateMutex, (TickType_t)0) == pdTRUE) {
                throttle_base_local = sharedState.throttle_base;
                xSemaphoreGive(xStateMutex);
        }

        float roll_cmd = torques_local.x;
        float pitch_cmd = torques_local.y;
        float yaw_cmd = torques_local.z;

        float motor1_out = throttle_base_local -roll_cmd + pitch_cmd;
        float motor2_out = throttle_base_local -roll_cmd - pitch_cmd;
        float motor3_out = throttle_base_local + roll_cmd + pitch_cmd;
        float motor4_out = throttle_base_local + roll_cmd - pitch_cmd;

        if (g_flags & FLAG_ARMED) {
            writePulseUs(0, floatToPulseUs(clampf(motor1_out, 0.0f, 1.0f)));
            writePulseUs(1, floatToPulseUs(clampf(motor2_out, 0.0f, 1.0f)));
            writePulseUs(2, floatToPulseUs(clampf(motor3_out, 0.0f, 1.0f)));
            writePulseUs(3, floatToPulseUs(clampf(motor4_out, 0.0f, 1.0f)));
        } else {
    // Desarmado: Pulso Mínimo (1000us)
        for (int i = 0; i < 4; i++) writePulseUs(i, PULSE_MIN_US);
        }
    vTaskDelayUntil(&xLastWakeTime, xFrequencyAttitude);
    }
}

// ==========================================================
//    TAREA 2: CONTROL DE ALTITUD (CORE 1 - BAJA PRIORIDAD)
// ==========================================================
void Task_AltitudeControl(void *pvParameters) {
    Serial.println("Tarea de Altitud iniciada en Core 1.");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float dt = 1.0f / ALTITUDE_LOOP_HZ;
    auto median3 = [](float a, float b, float c){ 
        if (a > b) { float t=a; a=b; b=t; } if (b > c) { float t=b; b=c; c=t; } if (a > b) { float t=a; a=b; b=t; } return b; 
    };

    for (;;) {
        VL53L0X_RangingMeasurementData_t m;
        lox.rangingTest(&m, false); float t1 = m.RangeMilliMeter;
        lox.rangingTest(&m, false); float t2 = m.RangeMilliMeter;
        lox.rangingTest(&m, false); float t3 = m.RangeMilliMeter;
        float tof_cm = 0.1f * median3(t1, t2, t3);
        // Fusión y Limitación
        const float alpha = 0.5f;
        float altitud_local = clampf(tof_cm, 0.0f, 300.0f); // 3 metros max

        // 2. Ejecutar Controlador PID
        float altitud_deseada = (float)g_setpoint_cm;
        float pid_correction = 0.0f;
        float throttle_base_local = THROTTLE_HOVER;

        if (g_flags & FLAG_ARMED) {
            pid_correction = altController->update(altitud_deseada, altitud_local, dt);
            pid_correction = clampf(pid_correction, 0.0f, 0.2f);
            throttle_base_local = THROTTLE_HOVER + pid_correction;
            Serial.printf("Alt setpoint: %d cm, medida: %.1f cm, throttle_base: %.2f\n",
                        g_setpoint_cm, altitud_local, throttle_base_local);
        }
        // 3. Actualizar el Estado Compartido (¡CON MUTEX!)
        if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
            sharedState.altitude_cm = altitud_local;
            sharedState.throttle_base = throttle_base_local;
            xSemaphoreGive(xStateMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequencyAltitude);
    }
}

// ==========================================================
//                   LÓGICA DE TELEMETRÍA (IOT)
//                     (CORE 0 - ASÍNCRONO)
// ==========================================================

// --- Variables de Red ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiUDP udpRx;
WiFiUDP udpTx;
static const uint16_t UDP_PORT_TX = 4000;
static const uint16_t UDP_PORT_RX = 4001;
static const IPAddress BROADCAST_IP(192,168,4,255);
static uint8_t g_seq = 0;
static IPAddress lastClientIP = IPAddress(0,0,0,0);

struct __attribute__((packed)) Telemetry {
        int16_t yaw_cdeg; 
        int8_t   pitch_d;
        int8_t   roll_d; 
        uint16_t alt_cm;
        uint8_t  flags; 
        uint8_t  seq;
};

static constexpr size_t TELEMETRY_SIZE = sizeof(Telemetry);
static uint8_t telemetryBuf[TELEMETRY_SIZE];
static Telemetry* telPtr = reinterpret_cast<Telemetry*>(telemetryBuf);

// --- Funciones de IOT ---

static void handleUdpRx() {
    int pkt = udpRx.parsePacket();
    if (pkt <= 0) return;
    uint8_t buf[64]; int r = udpRx.read(buf, sizeof(buf));
    if (r <= 0) return;
    g_last_heartbeat_ms = millis(); 
    if (r == 2) {
    int16_t sp = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1]<<8));
    g_setpoint_cm = constrain(sp, 0, 300);
    }
}

static void fillTelemetryFromSensors() {
    float r=0.0f, p=0.0f, y=0.0f, alt=0.0f;
    // Leer el estado compartido de forma segura
    if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
        r = sharedState.roll_deg;
        p = sharedState.pitch_deg;
        y = sharedState.yaw_deg;
        alt = sharedState.altitude_cm;
        xSemaphoreGive(xStateMutex);
    }   
    // Llenar el paquete de telemetría con datos escalados
    telPtr->yaw_cdeg = (int16_t)lrintf(clampf(y, -180.0f, 180.0f) * 100.0f);
    telPtr->pitch_d  = clamp_i8_deg(p);
    telPtr->roll_d   = clamp_i8_deg(r);
    telPtr->alt_cm   = (uint16_t)lrintf(clampf(alt, 0.0f, 65535.0f));
    telPtr->flags    = g_flags;
    telPtr->seq      = g_seq++;
}

static void sendTelemetry(const uint8_t *buf, size_t len) {
    if (ws.count() > 0) {
        ws.binaryAll((const char*)buf, len); 
        return; 
    }

    if (lastClientIP[0] != 0) { 
        udpTx.beginPacket(lastClientIP, UDP_PORT_TX); udpTx.write(buf, len); udpTx.endPacket(); return;
    }
    if (WiFi.softAPgetStationNum() > 0) {
        udpTx.beginPacket(BROADCAST_IP, UDP_PORT_TX); udpTx.write(buf, len); udpTx.endPacket();
    }
}

static void onWsEvent(AsyncWebSocket *serverPtr, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type){
        case WS_EVT_CONNECT:
            lastClientIP = client->remoteIP();
            Serial.printf("[WS] Cliente conectado: %s\n", lastClientIP.toString().c_str());
            client->client()->setNoDelay(true);
            break;

        case WS_EVT_DISCONNECT:
            if (g_flags & FLAG_ARMED) { 
                Serial.println("[FAILSAFE] Cliente desconectado. DESARMANDO.");
                g_flags &= ~FLAG_ARMED; 
            } else {
                Serial.println("[WS] Cliente desconectado.");
            }
            if (lastClientIP == client->remoteIP()) lastClientIP = IPAddress(0,0,0,0);
            break;

        case WS_EVT_DATA: { // <--- AQUI AGREGUE LA LLAVE DE APERTURA
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            
            // Solo procesamos si es un mensaje de texto completo y es el frame final
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                
                // Asegurar terminación nula
                data[len] = 0;
                String mensaje = (char*)data;
                
                int nuevo_setpoint = -1;
                
                // Lógica de parseo
                if (mensaje.startsWith("ALT:")) {
                    nuevo_setpoint = mensaje.substring(4).toInt();
                } else if (isdigit(mensaje.charAt(0))) {
                    nuevo_setpoint = mensaje.toInt();
                }

                if (nuevo_setpoint >= 0) {
                    g_setpoint_cm = constrain(nuevo_setpoint, 0, 300);
                    g_last_heartbeat_ms = millis(); 
                    Serial.printf("[WS] Setpoint recibido: %d cm\n", g_setpoint_cm);
                } else {
                    Serial.printf("[WS] Comando no reconocido: %s\n", mensaje.c_str());
                }
            }
            break; 
        } // <--- AQUI AGREGUE LA LLAVE DE CIERRE

        default: break;
    }
}

static void setupAP(){
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DRON_ESP32","12345678");
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
}

static void setupFS(){ if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); } }

static void setupServer(){
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/arm", HTTP_GET, [](AsyncWebServerRequest *request){
        g_last_heartbeat_ms = millis();
        g_flags |= FLAG_ARMED;
        Serial.println("[HTTP] ARMADO. Motores listos.");
        request->send(200, "text/plain", "ARMED");
    });

    server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest *request){
        g_flags &= ~FLAG_ARMED;
        Serial.println("[HTTP] DESARMADO. Motores detenidos.");
        request->send(200, "text/plain", "DISARMED");
    });

    server.on("/set_alt", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("cm")) {
            int cm = request->getParam("cm")->value().toInt();
            g_setpoint_cm = (int16_t)constrain(cm, 0, 300);
            g_last_heartbeat_ms = millis();
            Serial.printf("[HTTP] Setpoint actualizado: %d cm\n", g_setpoint_cm);
            request->send(200, "text/plain", "OK");
        } else {
            // Agrega este log para depurar si el cliente manda mal el parametro
            Serial.println("[HTTP ERROR] Petición a /set_alt sin parámetro 'cm'");
            
            // Listar qué parametros llegaron para ver cómo lo está enviando el dashboard
            int params = request->params();
            for(int i=0;i<params;i++){
                AsyncWebParameter* p = request->getParam(i);
                Serial.printf("Param recibido: %s = %s\n", p->name().c_str(), p->value().c_str());
            }
            request->send(400, "text/plain", "Missing cm parameter");
        }
    });

    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html").setCacheControl("no-cache");
    server.begin();
}

// ==========================================================
//                   CONFIGURACIÓN PRINCIPAL
// ==========================================================
void setup() {
    Serial.begin(115200);
    delay(100);
   Serial.println("Iniciando sistema DUAL-CORE...");

    // 1. Inicializar Sensores y Controladores
   Wire.begin(SDA_PIN, SCL_PIN, 400000); 
   if(!mpu.begin()) { 
        Serial.println("MPU6050 no responde."); 
        while(1) 
        delay(10); 
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    if(!mag.begin()) { 
        Serial.println("HMC5883L no responde."); 
        while(1) 
        delay(10); 
    }
    if(!lox.begin()) { 
        Serial.println("VL53L0X (ToF) no responde."); 
        while (1) 
        delay(10); 
    }
    lox.startRangeContinuous(); 
    Serial.println("Sensores inicializados.");

    // Inicializar Controladores
    ahrs = new MadgwickAHRS(0.1f);
    attitudeController = new LQRController();
    attitudeController->setGains(K_i_gains, K_q_gains, K_w_gains);
    altController = new AltitudePID();

    float alt_kp = 3.0f; // 3% de aceleración por cada cm de error
    float alt_ki = 0.005f; // Corrección lenta del error estacionario
    float alt_kd = 0.003f; // Amortiguación

    altController->setGains(alt_kp, alt_ki, alt_kd);         // PID
    altController->setAntiWindup(-0.2f, 0.2f);

    // 2. Setup de PWM (Motores)
    Serial.println("Configurando PWM y Armado de ESCs...");
    for (int i = 0; i < 4; i++) {
        ledcSetup(LEDC_CH[i], SERVO_FREQ_HZ, LEDC_RES_BITS);
        ledcAttachPin(PWM_PINS[i], LEDC_CH[i]);
        writePulseUs(i, PULSE_MIN_US);
    }
    delay(3000); 
    Serial.println("ESCs armados. Motores en IDLE (1000us).");

    // 3. Crear Mutex y Comunicación IoT (CORE 0)

    xStateMutex = xSemaphoreCreateMutex();
    if (xStateMutex == NULL) { 
        Serial.println("Error al crear Mutex!");
        while(1); 
    } else {
    setupAP();
    WiFi.setSleep(false);
    setupFS();
    setupServer();
    udpTx.begin(WiFi.softAPIP(), 0);
    size_t udp_status = udpRx.begin(UDP_PORT_RX); 
    if (udp_status != 1) { 
        Serial.println("[ERROR FATAL] Fallo al iniciar UDP RX en puerto 4001.");
        while(1); 
    }
    }
    
    // 4. Crear Tareas de FreeRTOS (Core 1 para Control)
    xTaskCreatePinnedToCore(Task_AttitudeControl, "Attitude", 10000, NULL, 2, NULL, 1); 
    xTaskCreatePinnedToCore(Task_AltitudeControl, "Altitude", 4096, NULL, 2, NULL, 1);

    Serial.println("Control de vuelo en Core 1. Comunicaciones en Core 0.");
    Serial.println(">>> Accede a http://192.168.4.1 y usa /arm para activar el vuelo. <<<");
}


// ==========================================================
//             TAREA 3: BUCLE PRINCIPAL (CORE 0)
// ==========================================================
// --- TAREA 3: BUCLE PRINCIPAL (CORE 0) ---
void loop() {
    static uint32_t lastTelemetryTx = 0;
    uint32_t now = millis();
    static uint32_t lastCleanup = 0;
    const uint32_t CLEANUP_INTERVAL_MS = 1000;

    // 1. Recibir Comandos UDP
    handleUdpRx();

    // 2. Mantenimiento de Conexiones (Aislado al inicio del loop)
    // HACEMOS LA LIMPIEZA AHORA, ANTES DE ENVIAR LA TELEMETRÍA.
    if (now - lastCleanup >= CLEANUP_INTERVAL_MS) {
        lastCleanup = now;
        ws.cleanupClients(); 
    }
    
    // 3. Enviar Telemetría (20 Hz)
    bool is_client_present = (ws.count() > 0 || WiFi.softAPgetStationNum() > 0);
    if (is_client_present && (now - lastTelemetryTx >= TELEMETRY_INTERVAL_MS)) {
        lastTelemetryTx = now;
        fillTelemetryFromSensors();
        sendTelemetry(telemetryBuf, TELEMETRY_SIZE);
    }
    
    // Cede el tiempo de CPU, dando más espacio al scheduler
    vTaskDelay(pdMS_TO_TICKS(5)); 
}