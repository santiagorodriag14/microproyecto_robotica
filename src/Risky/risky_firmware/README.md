# Firmware de Risky

Firmware de la ESP32 principal del robot escalador Risky. `main.cpp` (entorno `risky`) es el firmware del robot, `risky_manual` es el mismo sin telemetría; los `bench_*` prueban cada módulo por separado.

## Estructura

```
include/config.h            pines, canales LEDC, calibración y umbrales (todo lo que se ajusta)
include/util.h              Periodic y halt(), compartidos por los bancos
include/telemetry.h         estado que el control comparte con la telemetría
include/secrets.example.h   plantilla de WiFi y agente para bench_ros
lib/RcInput                 ancho de pulso del FS-R6B por interrupción y normalización
lib/Drive                   MotorSide (2 TB6612 en paralelo) y DifferentialDrive (mezcla + rampa)
lib/Esc                     señal de 50 Hz para los ESC
lib/Thrust                  ThrustGate: perillas → nivel de turbina, con armado seguro
lib/Imu                     MPU-6050, filtro del vector gravedad y clasificador de superficie
lib/Tof                     VL53L0X sin bloqueo
lib/RosLink                 micro-ROS por WiFi con reconexión al agente
src/main.cpp                firmware principal: teleoperación manual de ruedas y turbinas
src/telemetry_ros.cpp       tarea en el núcleo 0: IMU, ToF y micro-ROS (entorno risky)
src/telemetry_off.cpp       telemetría vacía (entorno risky_manual)
src/cam.cpp                 ESP32-CAM: video MJPEG por HTTP (entorno cam)
src/bench_*.cpp             un programa por banco
```

Cada librería recibe sus pines por parámetro, así que solo `config.h` conoce el esquemático.

## Cómo correr un banco

```
pio run -e bench_rc -t upload -t monitor
```

En VS Code se elige el entorno en la barra inferior de PlatformIO.

**micro-ROS no compila en Windows.** `bench_ros` y `risky` se construyen en Ubuntu, en la misma máquina donde corre ROS 2. Los bancos 1 a 5, `risky_manual` y `cam` funcionan en Windows.

## Firmware principal (`risky`)

Teleoperación manual por radio, con telemetría por micro-ROS. El control corre en el núcleo 1 y no depende de ROS: si el WiFi o el agente se caen, el robot se sigue manejando igual.

| Tópico | Contenido |
|---|---|
| `/risky/imu/data_raw` | IMU a 20 Hz |
| `/risky/tof/range` | distancia a la superficie a ~20 Hz |
| `/risky/cmd_vel` | velocidad pedida a la base (estimada con `drive::kMaxWheelSpeedMps`) |
| `/risky/thrust` | `[delantera, trasera]` de 0 a 1 |

Al encender, deje el robot quieto unos 2 s mientras se calibra el giroscopio.

| Control | Canal | Efecto |
|---|---|---|
| Palanca derecha ↕ | CH2 | avance y retroceso |
| Palanca derecha ↔ | CH1 | giro (en diagonal: avanza girando) |
| Knob A | CH5 | turbina delantera |
| Knob B | CH6 | turbina trasera |

- Al encender, las turbinas no arrancan hasta que ambas perillas pasen por el mínimo.
- Palanca suelta: las ruedas frenan (freno corto del TB6612).
- Sin señal RC: las ruedas frenan y las turbinas sostienen la última potencia, para que el robot no caiga de la pared. Para apagarlas hay que recuperar la señal y bajar las perillas, o cortar SW1.

## Orden de pruebas

### 1. `bench_rc`: receptor

Antes de conectar: mida con el osciloscopio que el pulso de señal del FS-R6B no pase de 3,3 V. Va directo a la ESP32.

Mapeo (pines del esquemático v5): CH1 → GPIO34 giro y CH2 → GPIO35 avance (palanca derecha), CH5 → GPIO32 = Knob A (turbina delantera), CH6 → GPIO33 = Knob B (turbina trasera). CH3 y CH4 quedan sin conectar.

1. Lleve cada palanca y perilla a sus extremos y déjela en reposo. Copie el mínimo, el centro y el máximo de cada canal a `rc::kCh1`, `kCh2`, `kCh5` y `kCh6` en `config.h`. Para las perillas solo importan el mínimo y el máximo.
2. Revise las direcciones: palanca derecha hacia adelante y hacia la derecha y las perillas en sentido horario deben **aumentar** el ancho de pulso. Si alguno baja, inviértalo en el menú *Reverse* del radio.
3. **Apague el control** con el receptor encendido y anote qué pasa:
   - aparece `----`: el receptor deja de mandar pulsos y el firmware detecta la pérdida de señal;
   - los valores se congelan: el receptor repite lo último y el firmware **no** puede detectar la pérdida. Hay que avisar antes de la integración, porque cambia el diseño del failsafe.

### 2. `bench_drive`: ruedas

Robot sobre un soporte, con las ruedas al aire.

- Palanca derecha hacia adelante (CH2): los dos lados giran hacia adelante. Si un lado gira al revés, cambie `drive::kLeftInverted` o `drive::kRightInverted`.
- CH1 a la derecha: el lado izquierdo gira más rápido.
- La velocidad sube en rampa, en unos 0,5 s.
- Al apagar el control (si en el banco 1 apareció `----`): las ruedas frenan.

### 3. `bench_esc`: turbinas, **sin hélices**

Calibración del rango, una sola vez por ESC:

1. Con SW1 apagado, conecte la ESP32 y abra el monitor.
2. Envíe `max`.
3. Encienda SW1. El ESC pita para indicar que entró en calibración.
4. Envíe `min`. El ESC pita para confirmar.

Después pruebe `a 10`, `1 20`, `2 20` y luego cualquier texto para apagar. Si un motor gira al revés, intercambie dos de sus fases.

### 3b. `bench_thrust`: turbinas con el radio, **sin hélices**

Requiere los ESC calibrados con `bench_esc`.

1. Al encender con las perillas subidas, las turbinas no arrancan: el monitor muestra "esperando ambas perillas en 0".
2. Lleve ambas perillas al mínimo: queda armado. Knob A debe mover la turbina delantera y Knob B la trasera; si salen al revés, intercambie `thrust::kFront*` y `thrust::kRear*` en `config.h`.
3. Apague el control con las turbinas girando: mantienen la velocidad (es el comportamiento buscado en la pared). Si en el banco 1 los valores se congelaban, el firmware no detecta la pérdida y las turbinas siguen al último valor que mandó el receptor.

### 3c. `bench_i2c`: bus de la IMU y el ToF

Al arrancar muestra si SDA y SCL están en alto en reposo; una línea en bajo es un dispositivo trabado o un corto. Luego busca dispositivos cada segundo a 400 kHz y a 100 kHz. Deben aparecer siempre `0x29 VL53L0X` y `0x68 MPU-6050`.

- Aparecen a 100 kHz pero no siempre a 400 kHz: cables largos o pull-ups débiles; bajar `kI2cHz` a 100000 en `config.h`.
- El MPU-6050 aparece y desaparece: conexión floja o alimentación; revisar soldaduras, VCC y GND del módulo.
- No aparece nunca: cableado, dirección (AD0) o módulo dañado.

### 4. `bench_imu`: inclinación y superficie

Deje el robot quieto al arrancar mientras calibra el giroscopio. El banco imprime `WHO_AM_I`: si no es `0x68`, el GY-521 trae un clon, pero igual debería funcionar.

La IMU va en la PCB del segmento trasero: gire el robot completo a mano y verifique:

| Posición | Inclinación esperada | Superficie |
|---|---|---|
| Plano en el suelo | ~0° | suelo |
| Contra la pared | ~90° | pared |
| Boca abajo | ~180° | techo |
| Entre esas posiciones | otro valor | transicion |

Si la inclinación da ~180° con el robot plano, el chip está montado con z hacia abajo. Hay que avisar para invertir el eje.

Repita la prueba con el robot sujeto y las turbinas encendidas: la inclinación no debería moverse más de unos pocos grados por la vibración.

### 5. `bench_tof`: distancia a la superficie

El VL53L0X no mide bien por debajo de unos 30 mm. Móntelo de modo que quede a más de 3 cm de la superficie. Compare con una regla y verifique unas 20 medidas por segundo.

### 6. `bench_ros`: telemetría micro-ROS (en Ubuntu)

1. Copie `include/secrets.example.h` como `include/secrets.h` y llene la red WiFi y la IP del PC.
2. Inicie el agente en el PC:
   ```
   docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
   ```
3. Grabe el banco y verifique:
   ```
   ros2 topic hz /risky/imu/data_raw     # ~20 Hz
   ros2 topic echo /risky/tof/range
   ```
4. Detenga el agente y vuélvalo a iniciar: la ESP32 debe reconectarse sola.

La distribución de ROS 2 está en `board_microros_distro` del `platformio.ini` (por defecto `humble`). Debe coincidir con la del agente.

### 7. `cam`: ESP32-CAM

La cámara es una placa aparte (AI-Thinker). Sirve su video MJPEG por HTTP y un nodo en el PC lo publica en ROS 2. No pasa por micro-ROS porque el video no cabe en ese enlace. Usa la misma red WiFi de `include/secrets.h`.

1. Monte la ESP32-CAM en la placa base ESP32-CAM-MB y grabe:
   ```
   pio run -e cam -t upload -t monitor
   ```
   El monitor muestra la IP. Si la placa se reinicia sola al conectar el WiFi, la alimentación no alcanza: use otro cable o puerto USB.
2. En un navegador de la misma red abra `http://<IP>/` (o `http://risky-cam.local/`). Debe verse el video y el control deslizante debe encender el flash.
3. En Ubuntu, con el workspace compilado:
   ```
   ros2 run risky_tools camera_bridge --ros-args -p url:=http://<IP>:81/stream
   ros2 topic hz /risky/camera/image/compressed
   ros2 run rqt_image_view rqt_image_view     # elegir /risky/camera/image/compressed
   ```
   Sin el parámetro `url`, el nodo busca `risky-cam.local`.

Solo un cliente puede ver el video a la vez: cierre la pestaña del navegador antes de lanzar `camera_bridge`.
