# Microproyecto Robótica: Metrología de Alta Precisión de Grietas

Este repositorio contiene el código fuente, diseños de hardware y documentación para el proyecto de investigación: **"Metrología de alta precisión de grietas mediante láser ToF con compensación inercial en tiempo real para teleoperación"**.

Para más detalles teóricos y metodológicos, puedes consultar el documento principal **Microproyecto Robótica** y el archivo **anteproyecto_herramienta.md** ubicados en la carpeta `docs/`.

## 📌 Descripción del Proyecto

En el ámbito de los sistemas ciber-físicos y los ensayos no destructivos (END), la inspección remota de infraestructuras críticas mediante plataformas robóticas se ve fuertemente afectada por vibraciones mecánicas y perturbaciones dinámicas de los motores BLDC. 

Este proyecto propone un marco metodológico estructurado para resolver estas incertidumbres métricas, integrando:
- Adquisición de datos por **láser Time-of-Flight (ToF)**.
- Telemetría inercial de alta frecuencia (IMU).

**Objetivo Principal:** Reducir el error métrico por debajo de 0.05 mm y mantener una latencia de lazo cerrado inferior a 5 ms en entornos teleoperados.

## 📂 Estructura del Repositorio

```text
📦 repositorio-robotica-tof
├── 📂 docs/             # Documentación, "Microproyecto Robótica" y "anteproyecto_herramienta.md"
├── 📂 hardware/         # Diseños CAD 3D y esquemas de los circuitos de sincronización
├── 📂 src/              # Código fuente (El "cerebro" del proyecto)
│   ├── 📂 control/      # Cinemática, control de motores BLDC y estabilización PID
│   ├── 📂 sensores/     # Drivers del ToF y sincronización por hardware del IMU
│   ├── 📂 procesamiento/# Fusión de datos (EKF) y algoritmos de metrología
│   └── 📂 teleoperacion/# Control remoto, scripts de latencia e interfaz de usuario
└── 📂 tests/            # Scripts de pruebas unitarias y validación cruzada (R&R)
```

## 🛠️ Requisitos e Instalación

1. Clona este repositorio en tu máquina local:
   ```bash
   git clone https://github.com/santiagorodriag14/microproyecto_robotica.git
   cd repositorio-robotica-tof
   ```

2. Instala las dependencias necesarias:
   ```bash
   pip install -r requirements.txt
   ```

