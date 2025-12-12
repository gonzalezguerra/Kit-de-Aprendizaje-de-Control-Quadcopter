#pragma once
#include <math.h>

class AltitudePID {
private:
    // Ganancias
    float kp, ki, kd;

    // Límites
    float output_min, output_max;
    float integral_min, integral_max;

    // Estado interno
    float integral;
    float prev_error;
    bool first_run;

    float lastP_ = 0.0f;
    float lastI_ = 0.0f;
    float lastD_ = 0.0f;

public:
    // Constructor
    AltitudePID() : 
        kp(0), ki(0), kd(0),
        output_min(-1.0f), output_max(1.0f),
        integral_min(-1.0f), integral_max(1.0f),
        integral(0), prev_error(0), first_run(true) {}

    /**
     * @brief Establece las ganancias del PID.
     */
    void setGains(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    /**
     * @brief Establece los límites de la salida total del PID.
     */
    void setLimits(float min, float max) {
        output_min = min;
        output_max = max;
    }

    /**
     * @brief Establece los límites del término integral (Anti-Windup).
     */
    void setAntiWindup(float min, float max) {
        integral_min = min;
        integral_max = max;
    }

    /**
     * @brief Resetea el estado del integrador.
     */
    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
        first_run = true;
    }

    /**
     * @brief Calcula la salida del controlador.
     * @param setpoint El valor deseado (ej. 100cm).
     * @param measurement El valor medido (ej. 90cm).
     * @param dt El tiempo delta (delta-time) en segundos.
     * @return La salida de control (corrección).
     */
    float update(float setpoint, float measurement, float dt);

    /**
     * @brief Obtiene el último valor del término P.
     */
    float lastP() const { return lastP_; }

    /**
     * @brief Obtiene el último valor del término I.
     */
    float lastI() const { return lastI_; }

    /**
     * @brief Obtiene el último valor del término D.
     */
    float lastD() const { return lastD_; }
};