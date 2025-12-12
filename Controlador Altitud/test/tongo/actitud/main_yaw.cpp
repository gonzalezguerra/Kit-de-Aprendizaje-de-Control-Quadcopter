//main.cpp funcional 13-11-25. CONTROL DE ACTITUD

#include <Arduino.h>
#include <Wire.h>
#include "sensores.h"
#include "MadgwickAHRS.h"   // Filtro Madgwick
#include "LQRController.h"  // Controlador LQR
#include "AltitudePID.h"    // Controlador de Altura PID
#include "Ultrasonic.h"     // Sensor Ultrasónico

// clampf: Función auxiliar para limitar valores flotantes
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// --- Frecuencias de Lazo ---
// Frecuencia del lazo de actitud (150 Hz = 6.67ms). Rango entre 150 y 250 Hz
const int ATTITUDE_LOOP_HZ = 150;
const TickType_t xFrequencyAttitude = pdMS_TO_TICKS(1000 / ATTITUDE_LOOP_HZ);

// Frecuencia del lazo de altitud (40 Hz = 25ms)
const int ALTITUDE_LOOP_HZ = 40;
const TickType_t xFrequencyAltitude = pdMS_TO_TICKS(1000 / ALTITUDE_LOOP_HZ);

// --- Controladores y Filtros ---
MadgwickAHRS* ahrs;
LQRController* attitudeController;
AltitudePID* altController;

// --- Pines para sensores ---
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const int TRIG_PIN = 25;
const int ECHO_PIN = 26;

// ----------------------------- PWM ------------------------------

// Pines de motores (PWM)
static const uint8_t PWM_PINS[4] = {32, 27, 4, 2}; // M1, M2, M3, M4
static const uint8_t LEDC_CH[4] = {0, 1, 2, 3}; // Canales LEDC para PWM

// Parametros de PWM
static const uint8_t LEDC_RES_BITS = 16; // Resolución de 16 bits
static const uint32_t SERVO_FREQ_HZ = 50; // Frecuencia típica de ESCs (50 Hz)
static const uint16_t PULSE_MIN_US = 1000; // Minimo para que giren
static const uint16_t PULSE_MAX_US = 2000; // Maximo para que giren
static const uint16_t PULSE_IDLE_US = 1050; // IDLE

// AUX PWM
inline  uint32_t periodUs() {
    return 1000000UL / SERVO_FREQ_HZ;       // 20000us para 50Hz
}
inline uint32_t maxDuty() {
    return (1UL << LEDC_RES_BITS) - 1;      // 65535 para 16 bits
}

/**
 * @brief Convierte microsegundos de pulso a valor de duty cycle para PWM.
 */
uint32_t pulseToDuty(uint32_t pulse_us) {
    return (uint64_t)pulse_us * maxDuty() / periodUs();
}

/**
 * @brief Escribe un ancho de pulso en microsegundos a un canal de motor
 */
void writePulseUs(uint8_t motor_index, uint16_t us) {
    if (us < PULSE_MIN_US) us = PULSE_MIN_US;
    if (us > PULSE_MAX_US) us = PULSE_MAX_US;
    ledcWrite(LEDC_CH[motor_index], pulseToDuty(us));
}

/**
 * @brief Convierte la salida del mezclador a un pulso
 */
uint16_t floatToPulseUs(float value) {
    return PULSE_MIN_US + (uint16_t)(value * (PULSE_MAX_US - PULSE_MIN_US));
}

// ----------------------------- fin PWM ------------------------------

// Objetos de Sensores (Globales)
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Kill switch (Interruptor de emergencia)
volatile bool g_ESTOP_ACTIVADO = false;

// Matrices de Calibración (Globales)
//float g_b[3] = { -0.01f + 0.00056284f, -0.06f + 0.00180133f, -0.02f + 0.00563336f };
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

float a_calibrado[3];
float g_calibrado[3];
float m_calibrado[3];
float roll, pitch, yaw;

// GANANCIAS LQR (Globales)

/*Matrix3x3 K_i_gains = {{{0.01f, 0.0f, 0.0f}, 
                        {0.0f, 0.01f, 0.0f}, 
                        {0.0f, 0.0f, 0.005f}}};
Matrix3x3 K_q_gains = {{{4.5f, 0.0f, 0.0f}, 
                        {0.0f, 4.5f, 0.0f}, 
                        {0.0f, 0.0f, 1.0f}}};
Matrix3x3 K_w_gains = {{{0.5f, 0.0f, 0.0f}, 
                        {0.0f, 0.5f, 0.0f}, 
                        {0.0f, 0.0f, 0.05f}}};*/

Matrix3x3 K_i_gains = {{{0.00f, 0.0f, 0.0f}, 
                        {0.0f, 0.00f, 0.0f}, 
                        {0.0f, 0.0f, 0.005f}}};
Matrix3x3 K_q_gains = {{{0.0f, 0.0f, 0.0f}, 
                        {0.0f, 0.0f, 0.0f}, 
                        {0.0f, 0.0f, 4.5f}}};
Matrix3x3 K_w_gains = {{{0.0f, 0.0f, 0.0f}, 
                        {0.0f, 0.0f, 0.0f}, 
                        {0.0f, 0.0f, 0.5f}}};
// Estado Compartido
// Estos son los datos que las tareas se comunican entre sí.
// Deben ser protegidos por un "Mutex" para evitar corrupción de datos.
SemaphoreHandle_t xStateMutex;
struct SharedControlState {
    Quaternion q_current = {1.0f, 0.0f, 0.0f, 0.0f}; // Escrito por Tarea_Actitud
    Vector3f w_current = {0.0f, 0.0f, 0.0f};         // Escrito por Tarea_Actitud
    float altitude_cm = 0.0f;                        // Escrito por Tarea_Altitud
    float throttle_base = 0.0f;                      // Escrito por Tarea_Altitud
    Vector3f torques = {0.0f, 0.0f, 0.0f};           // Escrito por Tarea_Actitud
} sharedState;

// Constantes mills yaw
const uint32_t MAG_WARMUP_MS = 3500;
uint32_t mag_init_ms = 0;

// ==========================================================
//    TAREA 1: LAZO DE ACTITUD Y ESTABILIZACIÓN (200 Hz)
// ==========================================================
void Task_AttitudeControl(void *pvParameters) {
    Serial.println("Tarea de Actitud iniciada en Core 1.");
    TickType_t xLastWakeTime;
    const float dt = 1.0f / ATTITUDE_LOOP_HZ; // dt es constante
    mag_init_ms = millis();

    // Variables de estado
    sensors_event_t mag_raw; // Para el magnetómetro
    sensors_event_t a, g, t; // Eventos del MPU6050
    sensors_event_t m;       // Evento del HMC5883

    // Arrays para datos crudos (en m/s^2 y rad/s)
    float a_raw[3], g_raw[3], m_raw[3];
    
    // Variables finales para los filtros (en m/s^2 y rad/s)
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 1. Lectura de Sensores IMU
        mpu.getEvent(&a, &g, &t); // Lee Accel y Gyro
        mag.getEvent(&m);         // Lee Magnetómetro

        // Cargar datos crudos en arrays
        a_raw[0] = a.acceleration.x;
        a_raw[1] = a.acceleration.y;
        a_raw[2] = a.acceleration.z;
        
        g_raw[0] = g.gyro.x;
        g_raw[1] = g.gyro.y;
        g_raw[2] = g.gyro.z;

        m_raw[0] = m.magnetic.x;
        m_raw[1] = m.magnetic.y;
        m_raw[2] = m.magnetic.z;

        // --- 2. Aplicar Calibraciones (de sensores.cpp) ---
        ax = a_A[0][0] * (a_raw[0] - a_b[0]) + a_A[0][1] * (a_raw[1] - a_b[1]) + a_A[0][2] * (a_raw[2] - a_b[2]);
        ay = a_A[1][0] * (a_raw[0] - a_b[0]) + a_A[1][1] * (a_raw[1] - a_b[1]) + a_A[1][2] * (a_raw[2] - a_b[2]);
        az = a_A[2][0] * (a_raw[0] - a_b[0]) + a_A[2][1] * (a_raw[1] - a_b[1]) + a_A[2][2] * (a_raw[2] - a_b[2]);

        // Calibrar Giroscopio: (g_raw - b)
        gx = g_raw[0] - g_b[0];
        gy = g_raw[1] - g_b[1];
        gz = g_raw[2] - g_b[2];

        // Imprimir los valores calibrados GYRO
        //Serial.print("GYRO_CALIBRADO [gx, gy, gz]: ");
        //Serial.print(gx, 6); // Imprimir con 6 decimales
        //Serial.print(", ");
        //Serial.print(gy, 6);
        //Serial.print(", ");
        //Serial.println(gz, 6);

        // Calibrar Magnetómetro: A*(m_raw - b)
        mx = m_A[0][0] * (m_raw[0] - m_b[0]) + m_A[0][1] * (m_raw[1] - m_b[1]) + m_A[0][2] * (m_raw[2] - m_b[2]);
        my = m_A[1][0] * (m_raw[0] - m_b[0]) + m_A[1][1] * (m_raw[1] - m_b[1]) + m_A[1][2] * (m_raw[2] - m_b[2]);
        mz = m_A[2][0] * (m_raw[0] - m_b[0]) + m_A[2][1] * (m_raw[1] - m_b[1]) + m_A[2][2] * (m_raw[2] - m_b[2]);

        // ==========================================================
        // --- DEBUG: VERIFICAR DATOS DEL YAW (MAGNETOMETRO) ---
        //
        // Imprimir los datos CRUDOS (antes de calibrar)
        //Serial.print("M_RAW [x,y,z]: ");
        //Serial.print(m_raw[0]); Serial.print(", ");
        //Serial.print(m_raw[1]); Serial.print(", ");
        //Serial.print(m_raw[2]); Serial.print(",");

        // Imprimir los datos CALIBRADOS (lo que ve el filtro)
        //Serial.print("M_CAL [mx,my,mz]: ");
        //Serial.print(mx); Serial.print(", ");
        //Serial.print(my); Serial.print(", ");
        //Serial.print(mz); Serial.println();
        // ==========================================================

        // --- DEBUG: PASO 1 (ACELEROMETRO-GYRO)---
        //Serial.print("CALIBRADO [AX,AY,AZ]: ");
        //Serial.print(ax); Serial.print(", ");
        //Serial.print(ay); Serial.print(", ");
        //Serial.print(az); Serial.print(", ");
        //Serial.print("CALIBRADO [GX,GY,GZ]: ");
        //Serial.print(gx); Serial.print(", ");
        //Serial.print(gy); Serial.print(", ");
        //Serial.print(gz); Serial.println();
        // --- FIN DEBUG ---

        // --- 3. Ejecutar Filtro de Fusión ---
        ahrs->update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

        // 4. Obtener Setpoint y Estado Actual
        Quaternion q_desired = {1.0f, 0.0f, 0.0f, 0.0f}; // Hover
        Quaternion q_current_local;
        ahrs->getQuaternion(&q_current_local.w, &q_current_local.x, &q_current_local.y, &q_current_local.z);
        q_current_local.z = 0.0f;
        // anadimos bias a q current local
        float q_bias_w = 0.0f;
        float q_bias_x = -0.0f;
        float q_bias_y = -0.0f;
        //float q_bias_z = -0.11f;
        q_current_local.w += q_bias_w;
        q_current_local.x += q_bias_x;
        q_current_local.y += q_bias_y;
        //q_current_local.z += q_bias_z;

        // En los primeros 3.5 s el yaw vale 0
        //if (millis() - mag_init_ms < MAG_WARMUP_MS) {
        //    q_current_local.z = 0.0f;
        //}

        // --- DEBUG: PASO 2 ---
        //Serial.print("CUATERNION [w,x,y,z]:    ");
        //Serial.print(q_current_local.w); Serial.print(", ");
        //Serial.print(q_current_local.x); Serial.print(", "); // Roll
        //Serial.print(q_current_local.y); Serial.print(", "); // Pitch
        //Serial.print(q_current_local.z); Serial.println();   // Yaw
        //--- FIN DEBUG ---

        Vector3f w_current_local = {gx, gy, gz}; // Velocidad angular actual
        
        // --- 5. Ejecutar Controlador LQR ---
        // agregar if de que si roll pitch o yaw estan entre -0.01 y 0.01 no controlar
        //if ( (q_current_local.x > -0.02f &&  q_current_local.x < 0.02f) &&
        //    (q_current_local.y > -0.02f &&  q_current_local.y < 0.02f) &&
        //    (q_current_local.z > -0.02f &&  q_current_local.z < 0.02f) ) {
        //    Vector3f torques_local = {0.0f, 0.0f, 0.0f};
        //}
        Vector3f torques_local = attitudeController->update(q_current_local, q_desired, w_current_local, dt);
        // torques_local ahora tiene los torques de control (tau_x, tau_y, tau_z)

        // --- 6. Actualizar el Estado Compartido (¡CON MUTEX!) ---
        if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
            sharedState.q_current = q_current_local;
            sharedState.w_current = w_current_local;
            sharedState.torques = torques_local;
            xSemaphoreGive(xStateMutex);
        }

        // --- 7. Mezclador de Motores ---
        // El mezclador se ejecuta aquí (el lazo más rápido)
        float throttle_base_local;

        // Leer el throttle del estado compartido de forma segura
        if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
            throttle_base_local = sharedState.throttle_base;
            xSemaphoreGive(xStateMutex);
        }

        // Renombrar torques para claridad
        float roll_cmd = torques_local.x;
        float pitch_cmd = torques_local.y;
        float yaw_cmd = torques_local.z;

        // Escribimos la logica del mezclador, esta en X
        /*float motor1_out = throttle_base_local - roll_cmd - pitch_cmd - yaw_cmd;
        float motor2_out = throttle_base_local - roll_cmd + pitch_cmd + yaw_cmd;
        float motor3_out = throttle_base_local + roll_cmd + pitch_cmd - yaw_cmd;
        float motor4_out = throttle_base_local + roll_cmd - pitch_cmd + yaw_cmd;*/

        // Funcionando si que si
        float motor1_out = -roll_cmd + pitch_cmd;
        float motor2_out = -roll_cmd - pitch_cmd;
        float motor3_out =  roll_cmd + pitch_cmd;
        float motor4_out =  roll_cmd - pitch_cmd;

        // Funcionando si que si CON YAW
        /*float motor1_out = -roll_cmd + pitch_cmd + yaw_cmd;
        float motor2_out = -roll_cmd - pitch_cmd - yaw_cmd;
        float motor3_out =  roll_cmd + pitch_cmd - yaw_cmd;
        float motor4_out =  roll_cmd - pitch_cmd + yaw_cmd;*/

        // Limitar salidas a 0.0 - 1.0
        motor1_out = clampf(motor1_out, 0.0f, 1.0f);
        motor2_out = clampf(motor2_out, 0.0f, 1.0f);
        motor3_out = clampf(motor3_out, 0.0f, 1.0f);
        motor4_out = clampf(motor4_out, 0.0f, 1.0f);

        // Limitar por seguridad a 0.0 - 0.7 para pruebas
        /*motor1_out = clampf(motor1_out, 0.0f, 0.7f);
        motor2_out = clampf(motor2_out, 0.0f, 0.7f);
        motor3_out = clampf(motor3_out, 0.0f, 0.7f);
        motor4_out = clampf(motor4_out, 0.0f, 0.7f);*/

        if (g_ESTOP_ACTIVADO) {
            // Si el ESTOP está activo, forzar TODOS los motores a 0% (1000us)
            writePulseUs(0, PULSE_MIN_US);
            writePulseUs(1, PULSE_MIN_US);
            writePulseUs(2, PULSE_MIN_US);
            writePulseUs(3, PULSE_MIN_US);
        }
            else{
            writePulseUs(0, floatToPulseUs(motor1_out));    // M1 -> Motor trasero derecho
            writePulseUs(1, floatToPulseUs(motor2_out));    // M2 -> Motor delantero derecho
            writePulseUs(2, floatToPulseUs(motor3_out));    // M3 -> Motor trasero izquierdo
            writePulseUs(3, floatToPulseUs(motor4_out));    // M4 -> Motor delantero izquierdo
        }
        
        // Print debug de salidas de motores PWM de cada una
        Serial.print("MOTOR_OUT [M1,M2,M3,M4]: ");
        Serial.print(motor1_out); Serial.print(", ");
        Serial.print(motor2_out); Serial.print(", ");
        Serial.print(motor3_out); Serial.print(", ");
        Serial.print(motor4_out); Serial.print(" | ");
        Serial.print("CUATERNION [w,x,y,z]: ");
        Serial.print(q_current_local.w); Serial.print(", ");
        Serial.print(q_current_local.x); Serial.print(", "); // Roll
        Serial.print(q_current_local.y); Serial.print(", "); // Pitch
        Serial.print(q_current_local.z); Serial.println();   // Yaw

        // Esta función duerme la tarea por el tiempo *exacto* que falta 
        // para completar el ciclo de 1/f ms.
        vTaskDelayUntil(&xLastWakeTime, xFrequencyAttitude);
    }
}

// ==========================================================
//    TAREA 2: LAZO DE ALTITUD (40 Hz)
// ==========================================================
/*void Task_AltitudeControl(void *pvParameters) {
    Serial.println("Tarea de Altitud iniciada en Core 1.");
    TickType_t xLastWakeTime;
    const float dt = 1.0f / ALTITUDE_LOOP_HZ; // dt es constante
    const float THROTTLE_HOVER = 0.5f;        // Placeholder, para flotar en hover
    
    // (Filtro de mediana de sensores.cpp)
    auto median3 = [](float a, float b, float c){ 
        if (a > b) { float t=a; a=b; b=t; } if (b > c) { float t=b; b=c; c=t; } if (a > b) { float t=a; a=b; b=t; } return b; 
    };

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // --- 1. Leer Sensores Lentos (Altitud) ---
        // Estas lecturas pueden ser lentas, pero no importa,
        // porque están en su propia tarea y no bloquean la actitud.
        float u1 = ultrasonic.read(CM);
        float u2 = ultrasonic.read(CM);
        float u3 = ultrasonic.read(CM);
        float ultra_cm = median3(u1, u2, u3);

        VL53L0X_RangingMeasurementData_t m;
        lox.rangingTest(&m, false); float t1 = m.RangeMilliMeter;
        lox.rangingTest(&m, false); float t2 = m.RangeMilliMeter;
        lox.rangingTest(&m, false); float t3 = m.RangeMilliMeter;
        float tof_cm = 0.1f * median3(t1, t2, t3);

        // --- 2. Fusionar lecturas de altitud ---
        const float alpha = 0.5f;
        float altitud_local = alpha * ultra_cm + (1.0f - alpha) * tof_cm;

        // --- 3. Ejecutar Controlador de Altitud ---
        float altitud_deseada = 100.0f; // 1 metro
        float pid_correction = altController->update(altitud_deseada, altitud_local, dt);
        float throttle_base_local = THROTTLE_HOVER + pid_correction;

        throttle_base_local = clampf(throttle_base_local, 0.0f, 1.0f); // Limitar a 0%-100%

        // --- 4. Actualizar el Estado Compartido (¡CON MUTEX!) ---
        if (xSemaphoreTake(xStateMutex, (TickType_t)5) == pdTRUE) {
            sharedState.altitude_cm = altitud_local;
            sharedState.throttle_base = throttle_base_local;
            xSemaphoreGive(xStateMutex);
        }
        
        // --- 5. Debugging (Opcional) ---
        Serial.print("Alt: "); Serial.print(altitud_local);
        Serial.print(" | PID_Corr: "); Serial.print(pid_correction);
        Serial.print(" | Thr_Out: "); Serial.println(throttle_base_local);

        // --- 6. Esperar hasta el próximo ciclo ---
        vTaskDelayUntil(&xLastWakeTime, xFrequencyAltitude);
    }
}*/

// ==========================================================
//    CONFIGURACIÓN PRINCIPAL (SETUP)
// ==========================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN, 400000); // 400kHz I2C

    // --- 1. Inicializar Sensores ---

    if(!mpu.begin()){
        Serial.println("MPU6050 does not respond");
        while(1) delay(10);
    }
    else{
        Serial.println("MPU6050 is connected");
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    if(!mag.begin()) {
        Serial.println("Error HMC5883"); while (1) delay(10);
    }
    else {
        Serial.println("HMC5883 is connected");
    }
    
    // --- Inicializar VL53L0X ---
    if(!lox.begin()){
        Serial.println("VL53L0X (ToF) not detected");
        while (1) delay(10);
    }
    else{
        Serial.println("VL53L0X (ToF) is connected");
        // Iniciar modo continuo para lecturas rápidas
        lox.startRangeContinuous(); 
    }

    // --- 2. Inicializar Controladores ---
    ahrs = new MadgwickAHRS(0.1f);
    attitudeController = new LQRController();
    attitudeController->setGains(K_i_gains, K_q_gains, K_w_gains);

    //HOLS BEW!!! Comnté la sección de altcontroller para probar unas cosas.
    // --- 3. Inicializar Controladores de Altitud ---
    //altController = new AltitudePID();

    // GANANCIAS PID de Altitud
    // Estas son ganancias de *ejemplo*. Probablemente necesiten mucho ajuste.
    // Sintoniza P primero, luego D, luego I.
    //float alt_kp = 0.008f; // 0.8% de aceleración por cada cm de error
    //float alt_ki = 0.002f; // Corrección lenta del error estacionario
    //float alt_kd = 0.005f; // Amortiguación
    //altController->setGains(alt_kp, alt_ki, alt_kd);

    // Limites de correccion del PID
    //altController->setLimits(-0.30f, 0.30f);

    // Anti-windup
    //altController->setAntiWindup(-0.15f, 0.15f);

    // --- 3. Crear Mutex ---
    xStateMutex = xSemaphoreCreateMutex();
    if(xStateMutex == NULL) {
        Serial.println("Error al crear Mutex!"); while(1);
    }

    Serial.println("Configurando canales LEDC para PWM...");
    for (int i = 0; i < 4; i++) {
        double realHz = ledcSetup(LEDC_CH[i], SERVO_FREQ_HZ, LEDC_RES_BITS);
        ledcAttachPin(PWM_PINS[i], LEDC_CH[i]);
        Serial.printf("M%d -> Pin %d, Canal %d, Freq %.2f Hz\n", i+1, PWM_PINS[i], LEDC_CH[i], realHz);
    }

    Serial.println("Armando ESCs: Enviando pulso mínimo (1000us) por 3 segundos...");
    for (int i = 0; i < 4; i++) {
        writePulseUs(i, PULSE_MIN_US);
    }

    delay(3000);
    Serial.println("ESCs armados.");

    // Opcional, usar el pulso IDLE
    // for (int i = 0; i < 4; i++) {
    //     writePulseUs(i, PULSE_IDLE_US);
    // }

    xStateMutex = xSemaphoreCreateMutex();

    // --- 4. Crear Tareas ---
    // Las tareas de control (tiempo real) deben ir en el Core 1.
    // El Core 0 puede manejar WiFi, Bluetooth, etc.
    
    xTaskCreatePinnedToCore(
        Task_AttitudeControl,   // Función de la tarea
        "Attitude Task",        // Nombre (para debug)
        10000,                  // Tamaño de la pila (stack)
        NULL,                   // Parámetros de la tarea
        2,                      // Prioridad (2 = alta)
        NULL,                   // Handle de la tarea
        1);                     // Core 1

    /*xTaskCreatePinnedToCore(
        Task_AltitudeControl,   // Función de la tarea
        "Altitude Task",        // Nombre
        4096,                   // Tamaño de la pila
        NULL,                   // Parámetros
        1,                      // Prioridad (1 = media/baja)
        NULL,                   // Handle
        1);                     // Core 1
    */

    // Kill switch. Core 0
    Serial.println("\n*** Tareas de control iniciadas en Core 1. ***");
    Serial.println("*** Monitor de seguridad (ESTOP) iniciado en Core 0. ***");
    Serial.println(">>> ¡Presiona 's' en cualquier momento para activar el ESTOP! <<<");
}

// ==========================================================
//    TAREA 3: MONITOR DE SEGURIDAD (se ejecuta en Core 0)
// ==========================================================
void loop() {
    // Si el ESTOP ya está activo, no hacer nada más.
    if (g_ESTOP_ACTIVADO) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Dormir 1 segundo
        return;
    }

    // Revisar si hay un comando serial
    if (Serial.available() > 0) {
        char key = Serial.read();
        
        // Usamos 's' (de Stop) como tecla de pánico
        if (key == 's' || key == 'S') {
            Serial.println("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("!!!     MOTORES DESARMADOS          !!!");
            Serial.println("!!! Reprogramar ESP32 Para Rearmar. !!!");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
            
            // Activa la flag de ESTOP
            g_ESTOP_ACTIVADO = true; 
        }
    }

    // Dormir esta tarea por un corto tiempo para no consumir el CPU
    vTaskDelay(pdMS_TO_TICKS(250)); // Revisar el serial 4 veces/seg
}