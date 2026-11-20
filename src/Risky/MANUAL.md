# Manual de usuario de Risky

Guía para encender el robot, lanzar los nodos de ROS 2 y manejarlo con el radio, en la simulación o con los dos a la vez. Los detalles de cada banco de pruebas del firmware están en `risky_firmware/README.md`, y los del modelo y la simulación en `risky_ws/README.md`.

## Contenido

1. [Qué hay en el sistema](#1-qué-hay-en-el-sistema)
2. [Preparación (solo la primera vez)](#2-preparación-solo-la-primera-vez)
3. [Lista de chequeo antes de cada sesión](#3-lista-de-chequeo-antes-de-cada-sesión)
4. [Encender el robot](#4-encender-el-robot)
5. [Lanzar los nodos](#5-lanzar-los-nodos)
6. [El panel de datos](#6-el-panel-de-datos)
7. [Manejar el robot con el radio](#7-manejar-el-robot-con-el-radio)
8. [Usar solo la simulación](#8-usar-solo-la-simulación)
9. [Comandos para verificar que todo funciona](#9-comandos-para-verificar-que-todo-funciona)
10. [Apagar](#10-apagar)
11. [Solución de problemas](#11-solución-de-problemas)
12. [Referencia de tópicos](#12-referencia-de-tópicos)
13. [Limitaciones conocidas](#13-limitaciones-conocidas)

---

## 1. Qué hay en el sistema

| Parte | Qué hace |
|---|---|
| **ESP32 principal** (firmware `risky`) | Lee el radio y maneja las ruedas y las turbinas. Además lee la IMU (MPU-6050) y el ToF (VL53L0X) y los publica en ROS 2 por micro-ROS. El control no depende de ROS: si se cae el WiFi, el robot se sigue manejando. |
| **ESP32-CAM** (firmware `cam`) | Transmite video MJPEG por HTTP. No es un nodo ROS. |
| **Radio FS-R6B** | Maneja el robot. Palanca derecha para las ruedas, perillas para las turbinas. |
| **PC con Ubuntu 22.04 y ROS 2 Humble** | Corre el agente micro-ROS, el puente de la cámara, el panel, Gazebo y RViz. |
| **Hotspot del celular** | La red que une el PC, el ESP32 y la ESP32-CAM (172.20.10.x). |

Carpetas:

```
~/Risky/
├── risky_firmware/   firmware de la ESP32 y de la ESP32-CAM (PlatformIO)
├── risky_ws/         workspace de ROS 2 (risky_description, risky_gazebo, risky_tools)
└── MANUAL.md         este archivo
~/microros_ws/        agente micro-ROS
```

---

## 2. Preparación (solo la primera vez)

### 2.1 Paquetes del PC

```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-joint-state-publisher-gui python3-pyqt5
```

El agente micro-ROS ya está compilado en `~/microros_ws`.

### 2.2 Compilar el workspace

```bash
cd ~/Risky/risky_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Con `--symlink-install`, los cambios en los archivos Python y en los launch se aplican sin volver a compilar. Hay que recompilar solo si se agrega un nodo o un archivo nuevo.

### 2.3 Red y firmware

1. En `risky_firmware/include/secrets.h` deben estar el nombre y la clave del hotspot, y la IP del PC en `kAgentIp`. Hoy esa IP es **172.20.10.4**.
2. Grabe el firmware del robot desde Ubuntu, porque micro-ROS no compila en Windows:
   ```bash
   cd ~/Risky/risky_firmware
   pio run -e risky -t upload
   ```
3. Grabe la cámara y anote la IP que muestra el monitor serie:
   ```bash
   pio run -e cam -t upload -t monitor
   ```

### 2.4 Que el PC no se cambie de red solo

Si el PC salta a la red de la universidad en medio de una sesión, se pierde la conexión con el robot y los nodos dejan de verse entre ellos. Para evitarlo:

```bash
nmcli connection modify WiFi-UAO connection.autoconnect no
```

Para revertirlo, use el mismo comando con `yes`.

---

## 3. Lista de chequeo antes de cada sesión

- [ ] Baterías cargadas (robot y radio).
- [ ] Hélices bien apretadas, y nadie con las manos cerca de ellas.
- [ ] Perillas de las turbinas (Knob A y Knob B) **al mínimo**.
- [ ] Hotspot del celular encendido.
- [ ] PC conectado al hotspot y con la IP correcta:
  ```bash
  ip -br addr show wlp3s0      # debe decir 172.20.10.4
  ```
  Si el celular le dio otra IP, el ESP32 no va a encontrar el agente. Hay dos salidas: cambiar `kAgentIp` en `secrets.h` y volver a grabar, o reconectar hasta que el celular asigne la .4.

---

## 4. Encender el robot

1. Encienda el radio primero y después el robot.
2. **Deje el robot quieto y apoyado unos 2 s.** En ese tiempo se calibra el giroscopio. Si lo mueve, el giro va a marcar un valor falso constante hasta el próximo reinicio.
3. Encienda la ESP32-CAM.
4. Para armar las turbinas, lleve las dos perillas al mínimo. Es una protección: si al encender alguna perilla estaba subida, las turbinas no arrancan hasta pasar por cero.

---

## 5. Lanzar los nodos

### 5.1 Lanzamiento normal (robot real + simulación + panel)

En una terminal:

```bash
source ~/microros_ws/install/setup.bash
source ~/Risky/risky_ws/install/setup.bash
ros2 launch risky_tools robot.launch.py camera_url:=http://172.20.10.3:81/stream
```

Cambie `172.20.10.3` por la IP actual de la cámara. Se abren tres ventanas: Gazebo, RViz y el panel de datos.

Este launch arranca:

| Nodo | Para qué |
|---|---|
| `micro_ros_agent` | Recibe por UDP (puerto 8888) los datos del ESP32 y los pasa a ROS 2 |
| `camera_bridge` | Lee el video de la ESP32-CAM y lo publica en `/risky/camera/image/compressed` |
| `dashboard` | El panel de datos |
| Gazebo + `ros_gz_bridge` | La simulación y la traducción de sus tópicos a ROS 2 |
| `thrust_adapter` | Pasa los comandos del radio a la simulación (turbinas en newtons y `cmd_vel`) |
| `robot_state_publisher` | Publica el árbol de TF del modelo |
| `rviz2` | Visualización 3D |

Con el robot real activo, la simulación funciona como **gemelo**: el radio mueve el robot de verdad y también el de Gazebo. Los sensores simulados se publican aparte, en `/sim/risky/...`, para que no se mezclen con los reales.

### 5.2 Otros modos

| Qué quiere | Comando |
|---|---|
| Solo el robot real, sin Gazebo ni RViz | `ros2 launch risky_tools robot.launch.py sim:=false` |
| Solo la simulación | `ros2 launch risky_tools robot.launch.py agent:=false camera:=false` |
| Robot real sin cámara | `ros2 launch risky_tools robot.launch.py camera:=false` |
| Todo, pero sin RViz | `ros2 launch risky_tools robot.launch.py rviz:=false` |
| Solo la simulación, con el launch original | `ros2 launch risky_gazebo sim.launch.py` |
| Solo el modelo en RViz, con sliders | `ros2 launch risky_description display.launch.py` |

### 5.3 Argumentos de `robot.launch.py`

| Argumento | Por defecto | Qué controla |
|---|---|---|
| `agent` | `true` | Lanza el agente micro-ROS |
| `agent_port` | `8888` | Puerto UDP del agente (debe coincidir con `kAgentPort` del firmware) |
| `camera` | `true` | Lanza `camera_bridge` |
| `camera_url` | `http://172.20.10.3:81/stream` | Dirección del video de la ESP32-CAM |
| `sim` | `true` | Lanza Gazebo con el bridge |
| `rviz` | `true` | Lanza RViz |
| `max_thrust_n` | `20.0` | Empuje de cada turbina al 100 % en la simulación, en newtons |
| `dashboard` | `true` | Abre el panel |

Para ver la lista en la terminal: `ros2 launch risky_tools robot.launch.py --show-args`.

---

## 6. El panel de datos

```
┌──────────────────────────────┬──────────────┬──────────────────┐
│                              │ ToF          │ Inclinación      │
│                              │ 124 mm       │ Alabeo  Cabeceo  │
│      Video ESP32-CAM         ├──────────────┴──────────────────┤
│                              │ Aceleración (m/s²)   x  y  z    │
│                              ├─────────────────────────────────┤
│                              │ Giro (°/s)           x  y  z    │
│                              ├─────────────────────────────────┤
│ 640×480 · 20 fps             │ Distancia ToF (mm)              │
├──────────────────────────────┴─────────────────────────────────┤
│ ● IMU: 20.0 Hz   ● ToF: 20.0 Hz   ● Cámara: 18.5 Hz            │
└────────────────────────────────────────────────────────────────┘
```

- **Video.** Si no llega imagen durante 1 s, se oscurece y dice "Sin video".
- **ToF.** Distancia a la superficie en milímetros. "sin blanco" significa que el sensor no ve nada dentro de su rango (hasta 1,2 m). Por debajo de unos 30 mm mide mal.
- **Inclinación.** Alabeo y cabeceo calculados con el acelerómetro. Solo valen con el robot casi quieto, porque cualquier aceleración o vibración se suma a la gravedad. Debajo aparece `|a|`: si se aleja mucho de 9,8 m/s², no confíe en la inclinación de ese momento.
- **Gráficas.** Los últimos 10 s de aceleración (m/s²), giro (°/s) y distancia del ToF.
- **Barra inferior.** Frecuencia de cada fuente. Punto verde si llegan datos, rojo con "sin datos" si pasó más de 1 s sin recibir nada. Es lo primero que conviene mirar cuando algo no anda.

El panel también se puede abrir solo: `ros2 run risky_tools dashboard`.

---

## 7. Manejar el robot con el radio

| Control | Canal | Efecto |
|---|---|---|
| Palanca derecha ↕ | CH2 | Avanzar y retroceder |
| Palanca derecha ↔ | CH1 | Girar (en diagonal avanza girando) |
| Knob A | CH5 | Turbina delantera |
| Knob B | CH6 | Turbina trasera |

Comportamiento que conviene conocer:

- **Palanca suelta:** las ruedas frenan (freno corto del TB6612).
- **La velocidad sube en rampa** en unos 0,5 s, no de golpe.
- **Sin señal del radio:** las ruedas frenan, pero **las turbinas mantienen la última potencia**, para que el robot no se caiga de la pared. Para apagarlas hay que recuperar la señal y bajar las perillas, o cortar SW1.
- En el modo gemelo, Gazebo recibe los mismos comandos. El robot simulado avanza y gira igual que el real, y sus turbinas empujan con `max_thrust_n × nivel²` newtons.

Para pasar a la pared se necesita un empuje total de al menos m·g/μ. Con 2 kg y μ = 0,6 son unos 33 N.

---

## 8. Usar solo la simulación

1. Lance:
   ```bash
   source ~/Risky/risky_ws/install/setup.bash
   ros2 launch risky_tools robot.launch.py agent:=false camera:=false
   ```
2. En otra terminal, maneje con el teclado:
   ```bash
   source ~/Risky/risky_ws/install/setup.bash
   ros2 run risky_tools keyboard_teleop
   ```

| Tecla | Efecto |
|---|---|
| `w` / `x` | Avanzar / retroceder (cada pulsación suma 0,01 m/s, hasta 0,10) |
| `a` / `d` | Girar a la izquierda / derecha |
| `s` | Frenar |
| `t` / `b` | Subir / bajar las dos turbinas |
| `y` / `n` | Subir / bajar la turbina delantera |
| `u` / `m` | Subir / bajar la turbina trasera |
| `0` | Apagar las turbinas |
| `Ctrl+C` | Salir |

En este modo el panel muestra los sensores de Gazebo, incluida su cámara.

No use `keyboard_teleop` con el robot real conectado: el teclado y el ESP32 publicarían en el mismo tópico y los comandos se mezclarían.

---

## 9. Comandos para verificar que todo funciona

Abra otra terminal y haga `source ~/Risky/risky_ws/install/setup.bash`.

```bash
ros2 topic list | grep risky                     # tópicos disponibles
ros2 topic hz /risky/imu/data_raw                # ~20 Hz
ros2 topic hz /risky/tof/range                   # ~20 Hz
ros2 topic hz /risky/camera/image/compressed     # fps de la cámara
ros2 topic echo /risky/cmd_vel                   # mueva la palanca y mire si cambia
ros2 topic echo /risky/thrust                    # mueva las perillas
ros2 topic echo /risky/odom --field pose.pose.position   # posición del robot en Gazebo
ros2 run rqt_graph rqt_graph                     # mapa de nodos y tópicos
```

El ESP32 publica en modo *best effort*. Si un `echo` no muestra nada aunque `topic list` sí muestre el tópico, agregue `--qos-reliability best_effort`.

---

## 10. Apagar

1. Baje las dos perillas al mínimo y espere a que las turbinas se detengan.
2. `Ctrl+C` en la terminal del launch. Se cierran todos los nodos, el panel incluido.
3. Apague el robot y después el radio. Al revés, el robot quedaría unos segundos sin señal con las turbinas sosteniendo la última potencia.

---

## 11. Solución de problemas

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| IMU y ToF en rojo en el panel | El ESP32 no se conectó al agente | Revise que el PC esté en el hotspot con IP 172.20.10.4. En la terminal del launch debe aparecer `session established`. |
| `No se encontró micro_ros_agent` al lanzar | No se hizo el `source` del agente | `source ~/microros_ws/install/setup.bash` y vuelva a lanzar. |
| El agente da `bind error ... port: 8888` | Ya hay otro agente o launch corriendo | Cierre el launch anterior. Para ver quién usa el puerto: `ss -ulpn \| grep 8888`. |
| Cámara en rojo, `Sin video de http://...` | IP equivocada, o la cámara está en otra red | Busque la IP en el monitor serie o en el celular y relance con `camera_url:=...`. |
| Cámara en rojo y el navegador sí muestra el video | La ESP32-CAM atiende un solo cliente a la vez | Cierre la pestaña del navegador. |
| `ros2 topic echo /risky/cmd_vel` dice que el tópico no existe | El ESP32 no llega al agente | Igual que la primera fila. |
| Llega `cmd_vel` pero Gazebo y RViz no se mueven | Gazebo y el bridge no se encuentran, o el launch quedó colgado de una red anterior | Cierre todo y relance ya conectado al hotspot. |
| `Exception sending a multicast message: Network is unreachable` | Ignition intenta usar una red caída | `sim.launch.py` ya fija `IGN_IP=127.0.0.1`. Si lanza Gazebo a mano, exporte esa variable antes. |
| Se cambió el WiFi con el launch corriendo | ROS 2 no se recupera de un cambio de IP | Cierre con `Ctrl+C`, reconecte al hotspot y relance. |
| El giro marca un valor constante con el robot quieto | Se movió durante la calibración | Reinicie el robot y déjelo quieto 2 s. |
| La inclinación del panel salta al encender las turbinas | Vibración. El acelerómetro mide gravedad más vibración | Es normal. Fíjese en `\|a\|`: si se aleja de 9,8 m/s², el ángulo no es confiable. |
| Las turbinas no arrancan | No se armaron | Lleve las dos perillas al mínimo y vuelva a subirlas. |
| El robot en Gazebo vibra o sale disparado | Masa o inercia irreal en el xacro | Revise lo último que cambió en `risky.urdf.xacro`. |
| Gazebo se cierra o el ToF simulado no publica en una máquina virtual | Problema de renderizado | Cambie `ogre2` por `ogre` en `worlds/risky_course.sdf`, o pruebe con `LIBGL_ALWAYS_SOFTWARE=1`. |

---

## 12. Referencia de tópicos

| Tópico | Tipo | Publica | Frecuencia |
|---|---|---|---|
| `/risky/imu/data_raw` | `sensor_msgs/Imu` | ESP32 (aceleración en m/s², giro en rad/s, sin orientación) | 20 Hz |
| `/risky/tof/range` | `sensor_msgs/Range` | ESP32 (metros, `inf` sin blanco) | ~20 Hz |
| `/risky/cmd_vel` | `geometry_msgs/Twist` | ESP32 (velocidad pedida, estimada desde el radio) o `keyboard_teleop` | 20 Hz |
| `/risky/thrust` | `std_msgs/Float32MultiArray` | ESP32 o `keyboard_teleop`. `[delantera, trasera]` de 0 a 1 | 20 Hz |
| `/risky/camera/image/compressed` | `sensor_msgs/CompressedImage` | `camera_bridge` (JPEG) | según la cámara |
| `/risky/camera/image_raw` | `sensor_msgs/Image` | Gazebo, solo en modo simulación | 15 Hz |
| `/risky/odom` | `nav_msgs/Odometry` | Gazebo (DiffDrive) | ~30 Hz |
| `/sim/risky/...` | | Sensores de Gazebo en modo gemelo | |
| `/sim/cmd_vel`, `/sim/thrust_*_n` | | `thrust_adapter` hacia Gazebo | 20 Hz |
| `/tf`, `/tf_static`, `/joint_states` | | Gazebo y `robot_state_publisher` | |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (tiempo simulado) | un mensaje por paso de física |

Si escribe un nodo que se suscribe a tópicos del ESP32, use `qos_profile_sensor_data`. Con la QoS por defecto (*reliable*), ROS 2 no lo conecta con un publicador *best effort* y no llega nada.

---

## 13. Limitaciones conocidas

- **ToF real en RViz.** En modo gemelo, RViz no lo dibuja. El ToF llega con la hora real y la TF de Gazebo usa el tiempo simulado, así que RViz descarta el mensaje. El panel sí lo muestra.
- **Inclinación del panel.** Sale solo del acelerómetro, así que en movimiento no es confiable. El firmware tiene un filtro que combina giroscopio y acelerómetro (`GravityFilter`), pero no se usa en el firmware principal ni se publica.
- **CPU de `thrust_adapter`.** Corre con el reloj de Gazebo y procesa `/clock` a cientos de mensajes por segundo, lo que le cuesta más del 60 % de un núcleo.
- **IMU simulada a 100 Hz y real a 20 Hz.** Tenerlo en cuenta al comparar gráficas.
- **Copias del bridge en `/tmp`.** El modo gemelo deja una copia de `bridge.yaml` (`/tmp/risky_bridge_*.yaml`) cada vez que se lanza. Se pueden borrar sin problema.
- **Valores sin medir.** Masa de la carga, fricción de las ruedas, empuje real de las turbinas y posición de los sensores siguen estimados. La tabla "Valores que hay que medir" de `risky_ws/README.md` explica cómo medir cada uno. Hasta entonces, la simulación es solo orientativa.
