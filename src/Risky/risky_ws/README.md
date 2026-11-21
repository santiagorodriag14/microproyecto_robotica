# Workspace ROS 2 de Risky

Simulación en Gazebo Fortress y visualización en RViz2 para ROS 2 Humble (Ubuntu 22.04).

| Paquete | Contenido |
|---|---|
| `risky_description` | URDF/xacro, mallas, configuración de RViz y `display.launch.py` |
| `risky_gazebo` | mundo con suelo, pared y techo, bridge de tópicos y `sim.launch.py` |
| `risky_tools` | `keyboard_teleop` (emula el radio), `thrust_adapter` (entrada de la simulación: turbinas → newtons y `cmd_vel`), `camera_bridge`, `dashboard` y `robot.launch.py` |

## Instalación

```bash
sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-joint-state-publisher-gui
mkdir -p ~/risky_ws && cp -r src ~/risky_ws/      # copiar esta carpeta src
cd ~/risky_ws
colcon build --symlink-install
source install/setup.bash
```

`ros-humble-ros-gz` instala Gazebo Fortress, el que corresponde a Humble.

## Uso

```bash
# 1. Solo el modelo, con sliders para el pivote y las ruedas
ros2 launch risky_description display.launch.py

# 2. Simulación completa (Gazebo + RViz)
ros2 launch risky_gazebo sim.launch.py

# 3. En otra terminal: manejar con el teclado
ros2 run risky_tools keyboard_teleop
```

## Robot real

```bash
source ~/microros_ws/install/setup.bash     # para el agente micro-ROS
source ~/Risky/risky_ws/install/setup.bash
ros2 launch risky_tools robot.launch.py camera_url:=http://172.20.10.3:81/stream
```

Lanza el agente micro-ROS (UDP 8888), el puente de la ESP32-CAM, la simulación (Gazebo y RViz) y el panel con el video, la IMU y el ToF. La barra inferior del panel marca en rojo el tópico que deja de llegar.

Con el robot real activo, la simulación funciona como gemelo digital. `twin_estimator` estima la pose del robot real (orientación con un filtro de Mahony sobre la IMU, posición avanzando por el eje x del robot con la velocidad de `/risky/cmd_vel`) y la publica en `/risky/odom` y `/tf`. La IMU mide el segmento trasero: el nodo lo apoya sobre el suelo, la pared o el techo de `risky_course.sdf` (parámetros `wall_x`, `ceiling_z`) y deduce el ángulo de la articulación delantera del contacto de sus ruedas. En Gazebo, el modelo se lanza con `twin:=true` en el xacro (sin tracción ni empuje, con `VelocityControl` en cero y `JointPositionController` en la articulación). Se ubica con el servicio `/world/risky_course/set_pose` y la articulación con `/sim/twin/front_joint`. Los sensores simulados pasan a `/sim/risky/...` y RViz usa la hora del PC, así que dibuja el ToF real. El robot debe encenderse en el origen de Gazebo, y `ros2 service call /risky/twin/reset std_srvs/srv/Trigger` lo devuelve ahí.

| Argumento | Por defecto | |
|---|---|---|
| `agent`, `agent_port` | `true`, `8888` | agente micro-ROS |
| `camera`, `camera_url` | `true`, `http://172.20.10.3:81/stream` | ESP32-CAM |
| `sim`, `rviz`, `max_thrust_n` | `true`, `true`, `20.0` | Gazebo y RViz |
| `speed_scale` | `1.0` | distancia real / distancia del gemelo |
| `yaw0` | `0.0` | rumbo inicial del gemelo (`3.1416` para subir de reversa) |
| `dashboard` | `true` | panel |

Solo la simulación, con el panel mostrando sus sensores: `ros2 launch risky_tools robot.launch.py agent:=false camera:=false`. Solo el robot real: `sim:=false`.

La inclinación del panel sale del acelerómetro y solo es correcta con el robot casi quieto.

## Contrato de tópicos

El robot real (ESP32 por micro-ROS) y la simulación usan los mismos tópicos. Pasar de uno a otro es lanzar el agente micro-ROS en vez de Gazebo; RViz y los demás nodos no cambian.

| Tópico | Tipo | Real | Simulación |
|---|---|---|---|
| `/risky/cmd_vel` | `geometry_msgs/Twist` | ESP32 (desde el radio) | `keyboard_teleop` |
| `/risky/thrust` | `std_msgs/Float32MultiArray` `[delantera, trasera]` 0–1 | ESP32 | `keyboard_teleop` |
| `/risky/imu/data_raw` | `sensor_msgs/Imu` | ESP32 | IMU de Gazebo |
| `/risky/tof/range` | `sensor_msgs/Range` | ESP32 | gpu_lidar de 1 rayo |
| `/risky/odom`, `/tf`, `/joint_states` | | `twin_estimator` | DiffDrive y JointStatePublisher |

El ESP32 publica todo en *best effort*. Quien se suscriba debe usar `qos_profile_sensor_data`; con la QoS por defecto (*reliable*) ROS 2 no los conecta y no llega nada. Por eso `cmd_vel` pasa por `thrust_adapter` antes del bridge de Gazebo.

Diferencia conocida: la IMU de Gazebo publica una orientación y la real no (`orientation_covariance[0] = -1`). Ningún nodo debe depender de ese campo.

## Cambios respecto al export de SolidWorks

Si se vuelve a exportar, hay que repetir estos cambios en `risky.urdf.xacro`:

1. **Eje de las ruedas** `0 0 1` → `0 0 -1`. Con el eje original, una velocidad positiva hacía retroceder al robot y DiffDrive lo manejaba al revés.
2. **Cabeceo del pivote** `-0.0086` → `0`. Dejaba las ruedas delanteras 2 mm en el aire.
3. **Colisiones**: cilindros para las ruedas y cajas para los chasis en lugar de las mallas (el chasis frontal tiene 151 mil triángulos). Las hélices no tienen colisión.
4. **Masa faltante**: `payload_link` con `payload_mass` = 0,70 kg, para llegar a ~2,0 kg en total.
5. **Marcos nuevos**: `imu_link` en el segmento trasero y `tof_link` en el costado izquierdo del delantero.
6. El paquete pasó de catkin (ROS 1) a ament (ROS 2), y las ocho ruedas y las dos hélices se generan con macros.

## Valores que hay que medir

Los resultados de la simulación solo valen tanto como estos números:

| Parámetro | Dónde | Cómo medirlo |
|---|---|---|
| `payload_mass` | `risky.urdf.xacro` | pesar el robot completo y restar 1,30 kg |
| `wheel_mu` | `risky.urdf.xacro` | inclinar una tabla con el robot encima hasta que deslice: μ = tan(ángulo) |
| `max_thrust_n` | parámetro de `thrust_adapter` | robot sobre una balanza con una sola turbina al 100 %: (kg con turbina − kg apagada) × 9,81 |
| `imu_link`, `tof_link` | `risky.urdf.xacro` | posición y orientación reales, tomadas del CAD |
| `wheel_separation` | `risky.gazebo.xacro` | subirla si el robot gira menos de lo pedido (deslizamiento lateral del skid-steer) |

Para sostenerse en la pared se necesita un empuje total de al menos m·g/μ. Con 2 kg y μ = 0,6 son unos 33 N. En la simulación, con μ = 1: con 2 × 7 N el robot pasa a la pared pero no sube; con 2 × 10 N sube la pared; con 2 × 20 N (valor por defecto de `sim.launch.py`, argumento `max_thrust_n`) también pasa al techo y avanza por él. Con los valores medidos, la simulación dice si el diseño alcanza.

## Problemas comunes

- **No se ven las mallas en Gazebo**: el launch agrega `IGN_GAZEBO_RESOURCE_PATH`. Revise que se haya hecho `source install/setup.bash`.
- **Gazebo se cierra o el ToF no publica en una máquina virtual**: cambie `ogre2` por `ogre` en `worlds/risky_course.sdf`, o pruebe con `LIBGL_ALWAYS_SOFTWARE=1`.
- **`Exception sending a multicast message: Network is unreachable` y nada se mueve**: Gazebo y el bridge no se encuentran por la red. `sim.launch.py` ya fija `IGN_IP=127.0.0.1`; si lanza Gazebo a mano, exporte esa variable antes.
- **El robot vibra o sale disparado**: casi siempre es una inercia o una masa irreal. Revise primero lo que haya cambiado en el xacro.
