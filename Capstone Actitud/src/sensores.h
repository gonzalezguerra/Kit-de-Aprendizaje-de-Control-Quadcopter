#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_VL53L0X.h>
#include <Ultrasonic.h>

// Pines
extern const int SDA_PIN, SCL_PIN, TRIG_PIN, ECHO_PIN;

// Objetos de sensores
extern Adafruit_MPU6050 mpu;
extern Adafruit_HMC5883_Unified mag;
extern Ultrasonic ultrasonic;
extern Adafruit_VL53L0X lox;

// Eventos/raw
extern sensors_event_t a_raw, g_raw, temp_raw;
extern sensors_event_t mag_raw;

// Config/calibración
extern const float alpha;
extern float g, declinacion, dt;
extern uint32_t t_prev;
extern float g_b[3], a_b[3], m_b[3];
extern float a_A[3][3], m_A[3][3];

// Estado filtrado (calibrado, antes de la fusión)
extern float gx, gy, gz;
extern float ax, ay, az;
extern float mx, my, mz;

// --- OUTPUTS FINALES ---
// Cuaternión (w, x, y, z)
extern float q0, q1, q2, q3; 

// Ángulos de Euler (para logging/debugging)
extern float roll, pitch, yaw; 

// Altura
extern float distancia_cm;

// Funciones de la API
void sensores_setup();
void sensores_tick();