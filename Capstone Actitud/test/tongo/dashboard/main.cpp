#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <FS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <math.h>
#include "sensores.h"
#include "MadgwickAHRS.h"
#include "LQRController.h"
#include "AltitudePID.h"
#include "Ultrasonic.h"

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

const int ATTITUDE_LOOP_HZ = 150;
const TickType_t xFrequencyAttitude = pdMS_TO_TICKS(1000 / ATTITUDE_LOOP_HZ);
const int ALTITUDE_LOOP_HZ = 40;
const TickType_t xFrequencyAltitude = pdMS_TO_TICKS(1000 / ALTITUDE_LOOP_HZ);

MadgwickAHRS* ahrs;
LQRController* attitudeController;
AltitudePID* altController;

const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int TRIG_PIN = 25;
const int ECHO_PIN = 26;

static const uint8_t PWM_PINS[4] = {32, 27, 4, 2};
static const uint8_t LEDC_CH[4] = {0, 1, 2, 3};

static const uint8_t LEDC_RES_BITS = 16;
static const uint32_t SERVO_FREQ_HZ = 50;
static const uint16_t PULSE_MIN_US = 1000;
static const uint16_t PULSE_MAX_US = 2000;
static const uint16_t PULSE_IDLE_US = 1050;

inline uint32_t periodUs() {
    return 1000000UL / SERVO_FREQ_HZ;
}
inline uint32_t maxDuty() {
    return (1UL << LEDC_RES_BITS) - 1;
}

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

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float g_b[3] = {-0.01f, -0.06f, -0.01f};
float a_A[3][3] = {
    {1.001926f, -0.004764f, -0.002582f},
    {-0.004764f, 0.995361f, -0.002533f},
    {-0.002582f, -0.002533f, 0.981982f}
};
float a_b[3] = {0.272221f, 0.080887f, 0.273688f};
float m_A[3][3] = {
    {1.238522f, -0.001869f, 0.030271f},
    {-0.001869f, 1.200648f, 0.080779f},
    {0.030271f, 0.080779f, 1.193193f}
};
float m_b[3] = {8.115422f, -6.052105f, -4.868707f};

float a_calibrado[3];
float g_calibrado[3];
float m_calibrado[3];
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float altura_cm = 0.0f;

Matrix3x3 K_i_gains = {{{0.01f, 0.0f, 0.0f},
                        {0.0f, 0.01f, 0.0f},
                        {0.0f, 0.0f, 0.01f}}};
Matrix3x3 K_q_gains = {{{2.2811f, 0.0f, 0.0f},
                        {0.0f, 2.2811f, 0.0f},
                        {0.0f, 0.0f, 1.0968f}}};
Matrix3x3 K_w_gains = {{{1.0179f, 0.0f, 0.0f},
                        {0.0f, 1.0155f, 0.0f},
                        {0.0f, 0.0f, 1.0148f}}};

SemaphoreHandle_t xStateMutex;
struct SharedControlState {
    Quaternion q_current = {1.0f, 0.0f, 0.0f, 0.0f};
    Vector3f w_current = {0.0f, 0.0f, 0.0f};
    float altitude_cm = 0.0f;
    float throttle_base = 0.0f;
    Vector3f torques = {0.0f, 0.0f, 0.0f};
} sharedState;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiUDP udpRx;
WiFiUDP udpTx;

static const uint16_t UDP_PORT_TX = 4000;
static const uint16_t UDP_PORT_RX = 4001;
static const IPAddress BROADCAST_IP(192, 168, 4, 255);

#define FLAG_ARMED (1 << 0)

struct __attribute__((packed)) Telemetry {
    int16_t yaw_cdeg;
    int8_t pitch_d;
    int8_t roll_d;
    uint16_t alt_cm;
    uint8_t flags;
    uint8_t seq;
};

static inline int8_t clamp_i8_deg(float d) {
    if (d > 127.0f) return 127;
    if (d < -128.0f) return -128;
    return (int8_t)lrintf(d);
}

static constexpr size_t TELEMETRY_SIZE = sizeof(Telemetry);
static uint8_t telemetryBuf[TELEMETRY_SIZE];
static Telemetry* telPtr = reinterpret_cast<Telemetry*>(telemetryBuf);

volatile int16_t g_setpoint_cm = 120;
volatile uint8_t g_flags = FLAG_ARMED;
static uint8_t g_seq = 0;
static IPAddress lastClientIP(0, 0, 0, 0);

static inline float wrap_360(float deg) {
    float x = fmodf(deg, 360.0f);
    if (x < 0.0f) x += 360.0f;
    return x;
}

static void debugDumpTelemetry() {
    float r_deg = roll;
    float p_deg = pitch;
    float y_deg = yaw;
    float alt = altura_cm;
    float pkt_r = telPtr->roll_d;
    float pkt_p = telPtr->pitch_d;
    float pkt_y = telPtr->yaw_cdeg / 100.0f;
    int pkt_a = telPtr->alt_cm;
    (void)r_deg;
    (void)p_deg;
    (void)y_deg;
    (void)alt;
    (void)pkt_r;
    (void)pkt_p;
    (void)pkt_y;
    (void)pkt_a;
}

static inline void fillTelemetryFromSensors() {
    float yaw_deg = clampf(yaw, -180.0f, 180.0f);
    float pitch_deg = clampf(pitch, -128.0f, 127.0f);
    float roll_deg = clampf(roll, -128.0f, 127.0f);
    float alt_cm_f = altura_cm;
    if (alt_cm_f < 0.0f) alt_cm_f = 0.0f;
    if (alt_cm_f > 65535.0f) alt_cm_f = 65535.0f;
    telPtr->yaw_cdeg = (int16_t)lrintf(yaw_deg * 100.0f);
    telPtr->pitch_d = clamp_i8_deg(pitch_deg);
    telPtr->roll_d = clamp_i8_deg(roll_deg);
    telPtr->alt_cm = (uint16_t)lrintf(alt_cm_f);
    telPtr->flags = g_flags;
    telPtr->seq = g_seq++;
}

static void handleUdpRx() {
    int pkt = udpRx.parsePacket();
    if (pkt <= 0) return;
    uint8_t buf[64];
    int r = udpRx.read(buf, sizeof(buf));
    if (r <= 0) return;
    if (r == 2) {
        int16_t sp = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        if (sp < 0) sp = 0;
        if (sp > 300) sp = 300;
        g_setpoint_cm = sp;
    } else {
        int maxLen = (int)sizeof(buf) - 1;
        if (r > maxLen) r = maxLen;
        buf[r] = 0;
        char* pos = strstr((char*)buf, "sp=");
        if (pos) {
            long v = strtol(pos + 3, nullptr, 10);
            if (v >= 0 && v <= 300) {
                g_setpoint_cm = (int16_t)v;
            }
        }
    }
}

static void sendTelemetry(const uint8_t* buf, size_t len) {
    if (ws.count() > 0) {
        if (!ws.availableForWriteAll()) {
            return;
        }
        ws.binaryAll((const char*)buf, len);
        return;
    }
    if (lastClientIP[0] != 0) {
        udpTx.beginPacket(lastClientIP, UDP_PORT_TX);
        udpTx.write(buf, len);
        udpTx.endPacket();
        return;
    }
    if (WiFi.softAPgetStationNum() > 0) {
        udpTx.beginPacket(BROADCAST_IP, UDP_PORT_TX);
        udpTx.write(buf, len);
        udpTx.endPacket();
    }
}


static void onWsEvent(AsyncWebSocket* serverPtr, AsyncWebSocketClient* client,
                        AwsEventType type, void* arg, uint8_t* data, size_t len) {
    (void)serverPtr;
    (void)arg;
    (void)data;
    (void)len;

    switch (type) {
        case WS_EVT_CONNECT:
            lastClientIP = client->remoteIP();
            Serial.printf("[WS] Cliente conectado: %s\n", lastClientIP.toString().c_str());
            client->client()->setNoDelay(true);
            break;
        case WS_EVT_DISCONNECT:
            Serial.println("[WS] Cliente desconectado");
            if (lastClientIP == client->remoteIP()) {
                lastClientIP = IPAddress(0, 0, 0, 0);
            }
            break;
        default:
            break;
    }
}




static void setupAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("DRON_ESP32", "12345678");
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                        IPAddress(192, 168, 4, 1),
                        IPAddress(255, 255, 255, 0));
}

static void setupFS() {
    if (!LittleFS.begin()) {
        LittleFS.format();
        LittleFS.begin();
    }
}

static void setupServer() {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/arm", HTTP_GET, [](AsyncWebServerRequest* request) {
        g_flags |= FLAG_ARMED;
        Serial.println("[HTTP] Comando ARMADO recibido.");
        request->send(200, "text/plain", "ARMED");
    });
    server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest* request) {
        g_flags &= (uint8_t)(~FLAG_ARMED);
        Serial.println("[HTTP] Comando DESARMADO recibido.");
        request->send(200, "text/plain", "DISARMED");
    });
    server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("no-cache");
    server.begin();
}

void Task_AttitudeControl(void* pvParameters) {
    (void)pvParameters;
    Serial.println("Tarea de Actitud iniciada en Core 1.");
    TickType_t xLastWakeTime;
    const float dt = 1.0f / ATTITUDE_LOOP_HZ;
    sensors_event_t mag_raw;
    sensors_event_t a, g, t;
    sensors_event_t m;
    float a_raw[3], g_raw[3], m_raw[3];
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        mpu.getEvent(&a, &g, &t);
        mag.getEvent(&m);
        a_raw[0] = a.acceleration.x;
        a_raw[1] = a.acceleration.y;
        a_raw[2] = a.acceleration.z;
        g_raw[0] = g.gyro.x;
        g_raw[1] = g.gyro.y;
        g_raw[2] = g.gyro.z;
        m_raw[0] = m.magnetic.x;
        m_raw[1] = m.magnetic.y;
        m_raw[2] = m.magnetic.z;
        ax = a_A[0][0] * (a_raw[0] - a_b[0]) + a_A[0][1] * (a_raw[1] - a_b[1]) + a_A[0][2] * (a_raw[2] - a_b[2]);
        ay = a_A[1][0] * (a_raw[0] - a_b[0]) + a_A[1][1] * (a_raw[1] - a_b[1]) + a_A[1][2] * (a_raw[2] - a_b[2]);
        az = a_A[2][0] * (a_raw[0] - a_b[0]) + a_A[2][1] * (a_raw[1] - a_b[1]) + a_A[2][2] * (a_raw[2] - a_b[2]);
        gx = g_raw[0] - g_b[0];
        gy = g_raw[1] - g_b[1];
        gz = g_raw[2] - g_b[2];
        mx = m_A[0][0] * (m_raw[0] - m_b[0]) + m_A[0][1] * (m_raw[1] - m_b[1]) + m_A[0][2] * (m_raw[2] - m_b[2]);
        my = m_A[1][0] * (m_raw[0] - m_b[0]) + m_A[1][1] * (m_raw[1] - m_b[1]) + m_A[1][2] * (m_raw[2] - m_b[2]);
        mz = m_A[2][0] * (m_raw[0] - m_b[0]) + m_A[2][1] * (m_raw[1] - m_b[1]) + m_A[2][2] * (m_raw[2] - m_b[2]);
        ahrs->update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
        Quaternion q_desired = {1.0f, 0.0f, 0.0f, 0.0f};
        Quaternion q_current_local;
        ahrs->getQuaternion(&q_current_local.w, &q_current_local.x, &q_current_local.y, &q_current_local.z);
        float q_bias_w = 0.0f;
        float q_bias_x = 0.0f;
        float q_bias_y = 0.0f;
        float q_bias_z = 0.0f;
        q_current_local.w += q_bias_w;
        q_current_local.x += q_bias_x;
        q_current_local.y += q_bias_y;
        q_current_local.z += q_bias_z;
        float qw = q_current_local.w;
        float qx = q_current_local.x;
        float qy = q_current_local.y;
        float qz = q_current_local.z;
        float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        float roll_local = atan2f(sinr_cosp, cosr_cosp);
        float sinp = 2.0f * (qw * qy - qz * qx);
        float pitch_local;
        if (fabsf(sinp) >= 1.0f) {
            float sign = sinp >= 0.0f ? 1.0f : -1.0f;
            pitch_local = sign * (3.14159265f * 0.5f);
        } else {
            pitch_local = asinf(sinp);
        }
        float siny_cosp = 2.0f * (qw * qz + qx * qy);
        float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        float yaw_local = atan2f(siny_cosp, cosy_cosp);
        float rad_to_deg = 180.0f / 3.14159265f;
        roll = roll_local * rad_to_deg;
        pitch = pitch_local * rad_to_deg;
        yaw = yaw_local * rad_to_deg;
        Vector3f w_current_local = {gx, gy, gz};
        Vector3f torques_local;
        if (fabsf(q_current_local.x) < 0.03f &&
            fabsf(q_current_local.y) < 0.03f &&
            fabsf(q_current_local.z) < 0.03f) {
            torques_local = {0.0f, 0.0f, 0.0f};
        } else {
            torques_local = attitudeController->update(q_current_local, q_desired, w_current_local, dt);
        }
        if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
            sharedState.q_current = q_current_local;
            sharedState.w_current = w_current_local;
            sharedState.torques = torques_local;
            xSemaphoreGive(xStateMutex);
        }
        float throttle_base_local = 0.0f;
        if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
            throttle_base_local = sharedState.throttle_base;
            xSemaphoreGive(xStateMutex);
        }
        float roll_cmd = torques_local.x;
        float pitch_cmd = torques_local.y;
        float yaw_cmd = torques_local.z;

        float motor1_out = throttle_base_local - roll_cmd + pitch_cmd + yaw_cmd;
        float motor2_out = throttle_base_local - roll_cmd - pitch_cmd - yaw_cmd;
        float motor3_out = throttle_base_local + roll_cmd + pitch_cmd - yaw_cmd;
        float motor4_out = throttle_base_local + roll_cmd - pitch_cmd + yaw_cmd;

        motor1_out = clampf(motor1_out, 0.0f, 1.0f);
        motor2_out = clampf(motor2_out, 0.0f, 1.0f);
        motor3_out = clampf(motor3_out, 0.0f, 1.0f);
        motor4_out = clampf(motor4_out, 0.0f, 1.0f);

        if (!(g_flags & FLAG_ARMED)) {
            writePulseUs(0, PULSE_MIN_US);
            writePulseUs(1, PULSE_MIN_US);
            writePulseUs(2, PULSE_MIN_US);
            writePulseUs(3, PULSE_MIN_US);
        } else {
            writePulseUs(0, floatToPulseUs(motor1_out));
            writePulseUs(1, floatToPulseUs(motor2_out));
            writePulseUs(2, floatToPulseUs(motor3_out));
            writePulseUs(3, floatToPulseUs(motor4_out));
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequencyAttitude);
    }
}

void Task_AltitudeControl(void* pvParameters) {
    (void)pvParameters;
    Serial.println("Tarea de Altitud iniciada en Core 1.");
    TickType_t xLastWakeTime;
    const float dt = 1.0f / ALTITUDE_LOOP_HZ;
    const float THROTTLE_HOVER = 0.4f;
    auto median3 = [](float a, float b, float c) {
        if (a > b) {
            float t = a;
            a = b;
            b = t;
        }
        if (b > c) {
            float t = b;
            b = c;
            c = t;
        }
        if (a > b) {
            float t = a;
            a = b;
            b = t;
        }
        return b;
    };
    xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        float u1 = ultrasonic.read(CM);
        float u2 = ultrasonic.read(CM);
        float u3 = ultrasonic.read(CM);
        float ultra_cm = median3(u1, u2, u3);
        VL53L0X_RangingMeasurementData_t m;
        lox.rangingTest(&m, false);
        float t1 = m.RangeMilliMeter;
        lox.rangingTest(&m, false);
        float t2 = m.RangeMilliMeter;
        lox.rangingTest(&m, false);
        float t3 = m.RangeMilliMeter;
        float tof_cm = 0.1f * median3(t1, t2, t3);
        const float alpha = 0.3f;
        float altitud_local = alpha * ultra_cm + (1.0f - alpha) * tof_cm;
        float altitud_deseada = (float)g_setpoint_cm;
        float pid_correction = altController->update(altitud_deseada, altitud_local, dt);
        float throttle_base_local = THROTTLE_HOVER + pid_correction;
        throttle_base_local = clampf(throttle_base_local, 0.0f, 1.0f);
        altura_cm = altitud_local;
        if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
            sharedState.altitude_cm = altitud_local;
            sharedState.throttle_base = throttle_base_local;
            xSemaphoreGive(xStateMutex);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequencyAltitude);
    }
}

void setup() {
    Serial.begin(115200);
    //delay(100);
    Serial.println("Iniciando sistema...");
    setupAP();
    WiFi.setSleep(false);
    setupFS();
    setupServer();
    udpTx.begin(WiFi.softAPIP(), 0);
    udpRx.begin(UDP_PORT_RX);
    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    if (!mpu.begin()) {
        Serial.println("MPU6050 does not respond");
        while (1) {
            delay(10);
        }
    } else {
        Serial.println("MPU6050 is connected");
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    if (!mag.begin()) {
        Serial.println("Error HMC5883");
        while (1) {
            delay(10);
        }
    } else {
        Serial.println("HMC5883 is connected");
    }
    if (!lox.begin()) {
        Serial.println("VL53L0X (ToF) not detected");
        while (1) {
            delay(10);
        }
    } else {
        Serial.println("VL53L0X (ToF) is connected");
        lox.startRangeContinuous();
    }
    ahrs = new MadgwickAHRS(0.2f);
    attitudeController = new LQRController();
    attitudeController->setGains(K_i_gains, K_q_gains, K_w_gains);
    altController = new AltitudePID();
    float alt_kp = 0.008f;
    float alt_ki = 0.001f;
    float alt_kd = 0.005f;
    altController->setGains(alt_kp, alt_ki, alt_kd);
    altController->setLimits(-0.30f, 0.30f);
    altController->setAntiWindup(-0.15f, 0.15f);
    xStateMutex = xSemaphoreCreateMutex();
    if (xStateMutex == NULL) {
        Serial.println("Error al crear Mutex");
        while (1) {
        }
    }
    Serial.println("Configurando canales LEDC para PWM...");
    for (int i = 0; i < 4; i++) {
        double realHz = ledcSetup(LEDC_CH[i], SERVO_FREQ_HZ, LEDC_RES_BITS);
        ledcAttachPin(PWM_PINS[i], LEDC_CH[i]);
        Serial.printf("M%d -> Pin %d, Canal %d, Freq %.2f Hz\n", i + 1, PWM_PINS[i], LEDC_CH[i], realHz);
    }
    Serial.println("Armando ESCs: Enviando pulso minimo por 3 segundos...");
    for (int i = 0; i < 4; i++) {
        writePulseUs(i, PULSE_MIN_US);
    }
    delay(3000);
    Serial.println("ESCs armados.");
    xTaskCreatePinnedToCore(
        Task_AttitudeControl,
        "Attitude Task",
        10000,
        NULL,
        2,
        NULL,
        1);
    xTaskCreatePinnedToCore(
        Task_AltitudeControl,
        "Altitude Task",
        4096,
        NULL,
        2,
        NULL,
        1);
    Serial.println("");
    Serial.println("*** Tareas de control iniciadas en Core 1. ***");
    Serial.println("*** Dashboard HTTP/WebSocket en Core 0. ***");
}

static const uint32_t TELEMETRY_INTERVAL_MS = 50;

void loop() {
    static uint32_t lastTelemetryTx = 0;
    static uint32_t lastWsCleanup = 0;
    uint32_t now = millis();

    handleUdpRx();

    if (now - lastTelemetryTx >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTx = now;
        fillTelemetryFromSensors();
        debugDumpTelemetry();
        sendTelemetry(telemetryBuf, TELEMETRY_SIZE);
    }

    if (now - lastWsCleanup >= 1000) {
        lastWsCleanup = now;
        ws.cleanupClients();
    }

    delay(10);
}

