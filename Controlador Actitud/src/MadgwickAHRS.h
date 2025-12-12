#pragma once
#include <math.h>

class MadgwickAHRS {
private:
    // Ganancia del filtro (parámetro 'beta'). 
    float beta;

    // Cuaternión estimado (w, x, y, z)
    float q0, q1, q2, q3; 

    // Función auxiliar rápida para inversa de raíz cuadrada
    static float invSqrt(float x);

public:
    // Constructor
    MadgwickAHRS(float beta_gain = 0.1f);

    // Función de actualización 9-DOF (Giro, Accel, Mag)
    // Acepta 'dt' (tiempo desde la última actualización) en segundos.
    void update(float gx, float gy, float gz, 
                float ax, float ay, float az, 
                float mx, float my, float mz, 
                float dt);

    // Función de actualización 6-DOF (Giro, Accel)
    void updateIMU(float gx, float gy, float gz, 
                    float ax, float ay, float az, 
                    float dt);

    // Orientaciones

    // Devuelve el cuaternión
    void getQuaternion(float *w, float *x, float *y, float *z) {
        *w = q0;
        *x = q1;
        *y = q2;
        *z = q3;
    }
    
    // Devuelve ángulos de Euler (Roll, Pitch, Yaw) en radianes
    // Por si acaso
    float getRoll();
    float getPitch();
    float getYaw();
};