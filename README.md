# Proyecto Educativo de Control con ESP32 y UAV (DXF + Firmware + Materiales)

Este repositorio reúne todos los recursos necesarios para que cualquier estudiante pueda **aprender control automático de manera económica y práctica**, utilizando una **ESP32**, sensores estándar y una estructura de dron diseñada para corte láser.

El proyecto incluye:
- Diseño del frame en **DXF**
- Firmware completo para **PlatformIO**
- Lista detallada de componentes y costos
- Guías prácticas para montaje y pruebas
- Material para extender el proyecto en cursos de control

---

# 🧩 Descripción General

Este proyecto está diseñado como una plataforma accesible para experimentar con:

- Control PID  
- Limitaciones físicas del control (saturación, ruido, retardo)
- Integración de sensores (IMU, magnetómetro, ToF)
- Actuadores (motores brushless + ESC)
- Modelación y simulación
- Construcción de UAV educativo

---

# 🛠️ Componentes y Costos Estimados

La siguiente tabla representa los costos aproximados para replicar la plataforma (valores en CLP).

> Esta tabla proviene del diseño original en LaTeX del proyecto, convertida a formato Markdown para uso en GitHub.

### **Tabla de costos (Markdown)**

| **Componente** | **Costo Estimado (CLP)** | **Proveedor** |
|----------------|---------------------------|---------------|
| **Cuerpo UAV** |                           |               |
| Base madera 3 mm | $2.500 | Sodimac |
| Corte láser | $7.000 | Tago |
| Soportes PLA (impresión 3D) | ≈ $2.000 | Bibliotecas UC |
| **Alimentación y Actuadores** | | |
| Batería LiPo 11.1 V, 3000 mAh | $17.600 | Aliexpress |
| Motores brushless 2212 2200 KV (x4) | $36.000 | AFEL |
| Drivers ESC 30 A (x4) | $17.200 | Aliexpress |
| Conectores XT60 PCB | $7.500 | Aliexpress |
| Adaptadores XT60 a T | $8.000 | AFEL |
| Hélices fibra de carbono | $11.000 | AFEL |
| **Electrónica** | | |
| ESP32 (38 pines) | $5.000 | Aliexpress |
| MPU6050 (IMU) | $4.000 | AFEL |
| Magnetómetro HMC5883L | $3.500 | AFEL |
| Sensor distancia láser VL53L0X | $6.000 | AFEL |
| **TOTAL ESTIMADO** | **$127.300** | |
