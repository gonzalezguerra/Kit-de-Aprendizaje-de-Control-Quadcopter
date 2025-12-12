#include "AltitudePID.h"

// Función de ayuda para limitar un valor
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ( (v > hi) ? hi : v );
}

float AltitudePID::update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) {
        return 0.0f; // No hacer nada si el tiempo no ha avanzado
    }

    // 1. Calcular el error
    float error = setpoint - measurement;

    // 2. Término Proporcional
    float P = kp * error;

    // 3. Término Integral (con anti-windup)
    integral += error * dt * 0.999f;
    integral = clampf(integral, integral_min, integral_max); // Anti-windup
    float I = ki * integral;

    // 4. Término Derivativo (con filtro para ruido)
    float derivative = 0.0f;
    if (first_run) {
        first_run = false;
    } else {
        // Derivada del error (evita "derivative kick" usando la medida)
        derivative = (error - prev_error) / dt;
    }
    prev_error = error;
    float D = kd * derivative;

    // Last PID

    lastP_ = P;
    lastI_ = I;
    lastD_ = D;

    // 5. Calcular salida total y limitar
    float output = P + I + D;
    output = clampf(output, output_min, output_max);

    return output;
}