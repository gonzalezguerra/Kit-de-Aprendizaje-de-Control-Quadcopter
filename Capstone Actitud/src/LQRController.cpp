#include "LQRController.h"

LQRController::LQRController() {
    // Inicializa todo a cero por seguridad
    reset();
    Matrix3x3 zero_matrix;
    setGains(zero_matrix, zero_matrix, zero_matrix);
}

void LQRController::setGains(const Matrix3x3& ki, const Matrix3x3& kq, const Matrix3x3& kw) {
    K_i = ki;
    K_q = kq;
    K_w = kw;
}

void LQRController::reset() {
    integral_error.x = 0.0f;
    integral_error.y = 0.0f;
    integral_error.z = 0.0f;
}

Vector3f LQRController::update(const Quaternion& q_current, 
                            const Quaternion& q_desired, 
                            const Vector3f& w_current, 
                            float dt) 
{
    // --- 1. Calcular el Estado de Error (x) ---

    // q_e = q_desired_conjugado * q_current
    Quaternion q_err = q_multiply(q_conjugate(q_desired), q_current);

    // Forzar la ruta más corta (evita que el dron gire 359 grados)
    // Si q_err.w es negativo, todo el cuaternión se invierte.
    if (q_err.w < 0.0f) {
        q_err.w = -q_err.w;
        q_err.x = -q_err.x;
        q_err.y = -q_err.y;
        q_err.z = -q_err.z;
    }

    // Estado de Error de Actitud (x_q)
    // Para errores pequeños, q_err.w ~ 1, y la parte vectorial es 2*error_vector
    // Usamos directamente la parte vectorial de q_e, que es proporcional al error.
    Vector3f x_q;
    x_q.x = q_err.x;
    x_q.y = q_err.y;
    x_q.z = q_err.z;
    
    // Estado de Error de Vel. Angular (x_w)
    // Asumimos que la velocidad angular deseada es 0
    Vector3f x_w = w_current;

    // Estado de Error Integral (x_i)
    // Actualizar el integrador (con anti-windup)
    integral_error.x += x_q.x * dt * 0.99f;
    integral_error.y += x_q.y * dt * 0.99f;
    integral_error.z += x_q.z * dt * 0.99f;

    // Anti-Windup Clamping
    integral_error.x = fmaxf(fminf(integral_error.x, integral_max), -integral_max);
    integral_error.y = fmaxf(fminf(integral_error.y, integral_max), -integral_max);
    integral_error.z = fmaxf(fminf(integral_error.z, integral_max), -integral_max);

    // --- 2. Calcular la Ley de Control: u = -K*x ---
    // u = - (K_i * x_i + K_q * x_q + K_w * x_w)

    Vector3f u_i = m3x3_v3_multiply(K_i, integral_error);
    Vector3f u_q = m3x3_v3_multiply(K_q, x_q);
    Vector3f u_w = m3x3_v3_multiply(K_w, x_w);

    Vector3f u_total = v3_add(u_i, v3_add(u_q, u_w));
    
    // --- ¡NUEVO! Guardar para debug ---
    last_u_i = u_i;
    last_u_q = u_q;
    last_u_w = u_w;

    // Devolver el torque negado (u = -Kx)
    return v3_scale(u_total, -1.0f);
}

// --- Implementaciones de Funciones Auxiliares ---

Quaternion LQRController::q_conjugate(const Quaternion& q) {
    Quaternion qc;
    qc.w = q.w;
    qc.x = -q.x;
    qc.y = -q.y;
    qc.z = -q.z;
    return qc;
}

Quaternion LQRController::q_multiply(const Quaternion& q1, const Quaternion& q2) {
    Quaternion q_out;
    q_out.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    q_out.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    q_out.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    q_out.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return q_out;
}

Vector3f LQRController::m3x3_v3_multiply(const Matrix3x3& m, const Vector3f& v) {
    Vector3f v_out;
    v_out.x = m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z;
    v_out.y = m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z;
    v_out.z = m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z;
    return v_out;
}

Vector3f LQRController::v3_add(const Vector3f& v1, const Vector3f& v2) {
    Vector3f result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
    return result;
}

Vector3f LQRController::v3_scale(const Vector3f& v, float s) {
    Vector3f result;
    result.x = v.x * s;
    result.y = v.y * s;
    result.z = v.z * s;
    return result;
}