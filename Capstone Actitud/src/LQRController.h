#pragma once
#include <math.h>

// --- Definiciones de tipos para claridad ---

// Cuaternión (w, x, y, z)
struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

// Vector de 3 ejes (para torques, velocidades, errores)
struct Vector3f {
    float x;
    float y;
    float z;
};

// Matriz de ganancia de 3x3
struct Matrix3x3 {
    // Fila-Columna
    float m[3][3];
};

class LQRController {
public:
    LQRController();

    /**
     * @brief Establece las ganancias LQR pre-calculadas.
     * @param K_i Ganancia integral (Matriz 3x3)
     * @param K_q Ganancia de actitud/proporcional (Matriz 3x3)
     * @param K_w Ganancia de vel. angular/derivativa (Matriz 3x3)
     */
    void setGains(const Matrix3x3& K_i, const Matrix3x3& K_q, const Matrix3x3& K_w);

    /**
     * @brief Reinicia el integrador a cero.
     */
    void reset();

    /**
     * @brief Calcula la salida de control (torques)
     * * @param q_current El cuaternión actual (de MadgwickAHRS)
     * @param q_desired El cuaternión deseado (del control remoto/misión)
     * @param w_current La velocidad angular actual (del giroscopio, en rad/s)
     * @param dt El tiempo delta (delta-time) en segundos (de sensores.cpp)
     * @return Vector3f Los torques de control (tau_x, tau_y, tau_z)
     */
    Vector3f update(const Quaternion& q_current, 
                    const Quaternion& q_desired, 
                    const Vector3f& w_current, 
                    float dt);

// --- ¡NUEVO! Función para extraer datos de debug ---
    // Pasa referencias para rellenar los valores
    void getDebugComponents(Vector3f& ui_out, Vector3f& uq_out, Vector3f& uw_out) {
        ui_out = last_u_i;
        uq_out = last_u_q;
        uw_out = last_u_w;
    }

private:
    // --- Ganancias del Controlador (Matriz K = [K_i, K_q, K_w]) ---
    Matrix3x3 K_i; // Ganancia Integral
    Matrix3x3 K_q; // Ganancia Proporcional (Error de Actitud)
    Matrix3x3 K_w; // Ganancia Derivativa (Error de Vel. Angular)

    // --- Estado del Controlador ---
    Vector3f integral_error; // El estado x_i (integral del error de actitud)

    // --- Límites Anti-Windup (evita que el integrador crezca indefinidamente) ---
    float integral_max = 0.3f; 

    // --- ¡NUEVO! Variables para almacenar el último cálculo ---
    Vector3f last_u_i = {0,0,0};
    Vector3f last_u_q = {0,0,0};
    Vector3f last_u_w = {0,0,0};

    // --- Funciones auxiliares de Cuaterniones ---
    Quaternion q_conjugate(const Quaternion& q);
    Quaternion q_multiply(const Quaternion& q1, const Quaternion& q2);
    
    // --- Funciones auxiliares de Vectores ---
    Vector3f m3x3_v3_multiply(const Matrix3x3& m, const Vector3f& v);
    Vector3f v3_add(const Vector3f& v1, const Vector3f& v2);
    Vector3f v3_scale(const Vector3f& v, float s);
};