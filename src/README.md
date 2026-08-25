# 💻 Código Fuente (src/)

El "cerebro" del proyecto. Aquí se encuentra todo el código ejecutable, scripts y algoritmos estructurados de forma modular.

## Módulos Principales
*   **`/control`**: Algoritmos de cinemática inversa/directa, control de corriente de los motores BLDC y lazos de estabilización (PID).
*   **`/sensores`**: Drivers para la adquisición de datos del láser ToF y rutinas de sincronización temporal de alta frecuencia del IMU.
*   **`/procesamiento`**: Algoritmos matemáticos y de fusión sensorial, incluyendo la implementación del Filtro de Kalman Extendido (EKF) y la estimación de estado estocástico.
*   **`/teleoperacion`**: Scripts para gestionar la interfaz del operador, la transmisión de comandos remotos y el monitoreo de la latencia de lazo cerrado (< 5 ms).

## Notas para el equipo
Todo el código de desarrollo principal debe ir aquí. Recuerden documentar sus funciones y clases detallando qué parámetros reciben y qué retornan.