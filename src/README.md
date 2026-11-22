# Risky: software del robot escalador

Código del robot escalador teleoperado Risky, que se adhiere a superficies verticales e invertidas con el empuje de dos hélices y lleva un sensor de distancia ToF y una cámara para inspeccionar fisuras. Esta carpeta reúne todo lo relacionado con programación: el firmware de las ESP32, el espacio de trabajo de ROS 2 con la simulación y el gemelo digital, y el modelo URDF original exportado desde SolidWorks.

## Contenido

| Carpeta o archivo | Qué contiene |
|---|---|
| `risky_firmware/` | Firmware de la ESP32 principal y de la ESP32-CAM, en un proyecto de PlatformIO |
| `risky_ws/` | Espacio de trabajo de ROS 2 Humble: modelo, simulación en Gazebo, gemelo digital, panel de datos y lanzadores |
| `urdf_risky/` | Export original de SolidWorks (paquete catkin de ROS 1), conservado como referencia |
| `MANUAL.md` | Manual de uso: encender el robot, lanzar los nodos, manejarlo con el radio y resolver problemas |

`risky_firmware/` y `risky_ws/` tienen su propio README con el detalle. Este archivo da la visión de conjunto.

## Cómo encaja todo

```
Radio FS-T6 ──2,4 GHz──> receptor FS-R6B ──> ESP32 (núcleo 1: control de ruedas y turbinas)
                                               │
                                               └ ESP32 (núcleo 0: IMU, ToF, micro-ROS)
                                                    │  Wi-Fi (UDP)
ESP32-CAM ── MJPEG por HTTP ──┐                     ▼
                              ▼            agente micro-ROS en el PC
                       camera_bridge                │
                              └──────────> ROS 2 Humble ──> twin_estimator ──> RViz y Gazebo
                                                    └─────> panel de datos
```

- El control del robot nunca pasa por la red. El receptor entra directo a la ESP32, así que una caída del Wi-Fi o del agente afecta la telemetría y el gemelo digital, pero no el manejo.
- La cámara no usa micro-ROS porque el video no cabe en ese enlace. La ESP32-CAM sirve MJPEG por HTTP y el nodo `camera_bridge` lo publica en ROS 2.
- El robot real y la simulación publican y escuchan los mismos tópicos, de modo que se puede pasar de uno a otro sin cambiar RViz ni los demás nodos.

## Requisitos

| Parte | Requisito |
|---|---|
| Firmware | PlatformIO. Los entornos sin micro-ROS compilan en Windows y Ubuntu, los que usan micro-ROS solo en Ubuntu |
| ROS 2 | Ubuntu 22.04 con ROS 2 Humble y Gazebo Fortress (`ros-humble-ros-gz`) |
| Paquetes del PC | `ros-humble-xacro`, `ros-humble-joint-state-publisher-gui`, `python3-pyqt5` |
| Agente micro-ROS | Compilado en `~/microros_ws`, o la imagen `microros/micro-ros-agent:humble` de Docker |
| Red | Un hotspot que una el PC, la ESP32 y la ESP32-CAM |

## Inicio rápido

Instalar los paquetes y compilar el espacio de trabajo:

```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-joint-state-publisher-gui python3-pyqt5
cd risky_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Configurar la red del firmware y grabarlo desde Ubuntu:

```bash
cd risky_firmware
cp include/secrets.example.h include/secrets.h
pio run -e risky -t upload
```

En `secrets.h` van el nombre y la clave del Wi-Fi y la IP del PC donde corre el agente. Ese archivo contiene credenciales y no debe subirse al repositorio.

Lanzar el robot real con la simulación y el panel:

```bash
source ~/microros_ws/install/setup.bash
source risky_ws/install/setup.bash
ros2 launch risky_tools robot.launch.py camera_url:=http://<IP de la cámara>:81/stream
```

Solo la simulación, manejada con el teclado:

```bash
ros2 launch risky_tools robot.launch.py agent:=false camera:=false
ros2 run risky_tools keyboard_teleop
```

El paso a paso completo, con la lista de chequeo antes de cada sesión, está en `MANUAL.md`.

## risky_firmware

Firmware en C++ sobre el núcleo Arduino de la ESP32. Cada función vive en una librería independiente que recibe sus pines por parámetro, y solo `include/config.h` conoce el esquemático.

```
include/config.h       pines, canales LEDC, calibración del radio y umbrales
include/telemetry.h    estado que el control comparte con la telemetría
include/secrets.example.h   plantilla de Wi-Fi y dirección del agente
lib/RcInput            ancho de pulso del receptor por interrupción, mediana y normalización
lib/Drive              MotorSide (dos TB6612FNG en paralelo) y DifferentialDrive (mezcla y rampa)
lib/Esc                señal de 50 Hz para los ESC
lib/Thrust             ThrustGate: perillas a nivel de turbina, con armado seguro
lib/Imu                MPU-6050, filtro del vector gravedad y clasificador de superficie
lib/Tof                VL53L0X sin bloqueo
lib/RosLink            micro-ROS por Wi-Fi con reconexión al agente
src/main.cpp           firmware principal
src/telemetry_ros.cpp  tarea de telemetría en el núcleo 0
src/telemetry_off.cpp  telemetría vacía
src/cam.cpp            ESP32-CAM: video MJPEG por HTTP
src/bench_*.cpp        un banco de prueba por módulo
```

### Firmware principal

El lazo de control corre en el núcleo 1 cada 10 ms. Lee el radio, mueve las ruedas con mezcla diferencial y una rampa de 0,5 s, y fija el nivel de cada turbina. En el núcleo 0 corre una tarea aparte que lee el IMU y el ToF y publica por micro-ROS, de modo que el Wi-Fi nunca frena el control.

Control con el radio:

| Control | Canal | GPIO | Efecto |
|---|---|---|---|
| Palanca derecha, vertical | CH2 | 35 | Avance y retroceso |
| Palanca derecha, horizontal | CH1 | 34 | Giro |
| Knob A | CH5 | 32 | Turbina delantera |
| Knob B | CH6 | 33 | Turbina trasera |

Comportamiento de seguridad:

- Con la palanca suelta, las ruedas frenan (freno corto del TB6612FNG).
- Las turbinas no arrancan hasta que ambas perillas pasan por el mínimo.
- Si cualquier canal pasa 100 ms sin pulso válido, las ruedas frenan y las turbinas mantienen su último nivel, para que el robot no se caiga de la pared. Para apagarlas hay que recuperar la señal y bajar las perillas, o cortar la alimentación de la rama de propulsión.
- Al encender, el robot debe quedar quieto unos 2 s mientras se calibra el giroscopio.

### Entornos de PlatformIO

| Entorno | Para qué sirve | Se compila en |
|---|---|---|
| `risky` | Firmware principal con telemetría micro-ROS | Ubuntu |
| `risky_manual` | El mismo firmware sin telemetría | Windows y Ubuntu |
| `bench_rc` | Receptor: calibrar extremos y centro de cada canal | Windows y Ubuntu |
| `bench_drive` | Ruedas, con el robot sobre un soporte | Windows y Ubuntu |
| `bench_esc` | ESC y turbinas, sin hélices | Windows y Ubuntu |
| `bench_thrust` | Turbinas con el radio, sin hélices | Windows y Ubuntu |
| `bench_i2c` | Bus I2C del IMU y el ToF | Windows y Ubuntu |
| `bench_imu` | Inclinación y superficie detectada | Windows y Ubuntu |
| `bench_tof` | Distancia a la superficie | Windows y Ubuntu |
| `bench_ros` | Telemetría micro-ROS | Ubuntu |
| `cam` | ESP32-CAM | Windows y Ubuntu |

Para correr un banco:

```bash
pio run -e bench_rc -t upload -t monitor
```

Se recomienda probar los bancos en ese orden antes de grabar el firmware principal. El procedimiento de cada uno está en `risky_firmware/README.md`.

## risky_ws

Espacio de trabajo de ROS 2 Humble con tres paquetes:

| Paquete | Contenido |
|---|---|
| `risky_description` | Modelo en xacro, mallas, configuración de RViz y `display.launch.py` |
| `risky_gazebo` | Mundo de prueba (piso, pared y techo), puente de tópicos con Gazebo y `sim.launch.py` |
| `risky_tools` | Nodos en Python y el lanzador `robot.launch.py` |

Nodos de `risky_tools`:

| Nodo | Función |
|---|---|
| `twin_estimator` | Estima la pose del robot real y mueve con ella los modelos de Gazebo y RViz |
| `camera_bridge` | Publica el video de la ESP32-CAM como `sensor_msgs/CompressedImage` |
| `dashboard` | Panel con video, ToF, inclinación, gráficas de aceleración y giro, y frecuencia de cada fuente |
| `thrust_adapter` | En simulación, convierte el nivel de turbina (0 a 1) en newtons y reenvía `cmd_vel` |
| `keyboard_teleop` | Emula el radio con el teclado en la simulación |

### Lanzadores

| Qué se quiere | Comando |
|---|---|
| Robot real, simulación y panel | `ros2 launch risky_tools robot.launch.py` |
| Solo el robot real | `ros2 launch risky_tools robot.launch.py sim:=false` |
| Solo la simulación | `ros2 launch risky_tools robot.launch.py agent:=false camera:=false` |
| Simulación con el lanzador original | `ros2 launch risky_gazebo sim.launch.py` |
| Modelo en RViz con sliders | `ros2 launch risky_description display.launch.py` |

Argumentos principales de `robot.launch.py`:

| Argumento | Por defecto | Controla |
|---|---|---|
| `agent`, `agent_port` | `true`, `8888` | Agente micro-ROS |
| `camera`, `camera_url` | `true`, `http://172.20.10.3:81/stream` | Puente de la cámara |
| `sim`, `rviz`, `dashboard` | `true` | Gazebo, RViz y panel |
| `max_thrust_n` | `20.0` | Empuje de cada turbina al 100 % en la simulación |
| `speed_scale` | `1.0` | Factor entre la distancia real y la del gemelo |
| `yaw0` | `0.0` | Rumbo inicial del gemelo (`3.1416` para subir de reversa) |

### Tópicos

El robot real y la simulación comparten estos tópicos:

| Tópico | Tipo | Real | Simulación |
|---|---|---|---|
| `/risky/cmd_vel` | `geometry_msgs/Twist` | ESP32 | `keyboard_teleop` |
| `/risky/thrust` | `std_msgs/Float32MultiArray`, `[delantera, trasera]` de 0 a 1 | ESP32 | `keyboard_teleop` |
| `/risky/imu/data_raw` | `sensor_msgs/Imu` | ESP32 | IMU de Gazebo |
| `/risky/tof/range` | `sensor_msgs/Range` | ESP32 | Lidar de un rayo |
| `/risky/camera/image/compressed` | `sensor_msgs/CompressedImage` | `camera_bridge` | |
| `/risky/odom`, `/tf`, `/joint_states` | | `twin_estimator` | DiffDrive y JointStatePublisher |

Dos cosas que conviene saber al escribir un nodo nuevo:

- La ESP32 publica en modo best effort. Quien se suscriba debe usar `qos_profile_sensor_data`, porque con la calidad de servicio por defecto ROS 2 no conecta los dos extremos y no llega nada.
- El IMU de Gazebo publica una orientación y el real no (`orientation_covariance[0] = -1`). Ningún nodo debe leer ese campo.

### Gemelo digital

Cuando el agente y la simulación corren a la vez, `twin_estimator` mueve los modelos de Gazebo y RViz con la pose del robot real:

- **Orientación:** filtro complementario de Mahony sobre el IMU. El giroscopio da el giro y el acelerómetro corrige alabeo y cabeceo cuando mide solo gravedad. El rumbo se integra del giroscopio y se corrige con el contacto contra la pared.
- **Posición:** navegación a estima con la velocidad de `/risky/cmd_vel` a lo largo del eje x del robot. Como ese eje sale de la orientación, en la pared el avance se vuelve subida y en el techo vuelve a ser horizontal.
- **Contacto:** como el IMU va en el segmento trasero, el nodo lo apoya sobre el piso, la pared o el techo de `risky_course.sdf` y deduce el ángulo del pivote delantero del contacto de sus ruedas.

El robot debe encenderse en el origen de Gazebo y quedarse quieto unos segundos para calibrar el giroscopio y nivelar el IMU. Con `ros2 service call /risky/twin/reset std_srvs/srv/Trigger` el gemelo vuelve al punto de partida.

### Simulación

La física corre en Gazebo Fortress con el solver PGS de DART. Con el solver por defecto las ruedas patinan contra la pared y el segmento delantero nunca se levanta.

Si se vuelve a exportar el modelo desde SolidWorks, hay que repetir estos cambios en `risky.urdf.xacro`:

1. Invertir el eje de las ruedas, de `0 0 1` a `0 0 -1`.
2. Poner en cero el cabeceo del pivote, que dejaba las ruedas delanteras 2 mm en el aire.
3. Usar colisiones simples en lugar de mallas, porque el chasis delantero tiene 151 mil triángulos.
4. Agregar `payload_link` con la masa que el CAD no trae (batería y electrónica).
5. Agregar los marcos `imu_link` y `tof_link`.

## urdf_risky

Export original del SolidWorks URDF Exporter, con mallas STL, un URDF y los lanzadores de ROS 1. Se conserva como referencia y ya no se usa. Si se coloca dentro de `risky_ws`, `colcon build` falla buscando `catkin`, así que debe permanecer fuera del espacio de trabajo o llevar un archivo `COLCON_IGNORE`.


La lista completa y el procedimiento para medir cada valor están en `MANUAL.md` y en `risky_ws/README.md`.
