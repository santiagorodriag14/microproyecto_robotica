# Protocolo de verificación de Risky con los bancos de prueba

Este protocolo sirve para comprobar, módulo por módulo, que el hardware y el firmware de Risky funcionan antes de usar el firmware completo. Cada banco es un programa aparte en `src/`, con su propio entorno en `platformio.ini`, y prueba un solo subsistema. El orden va de lo más simple a lo integrado: si un banco falla, los siguientes no son confiables hasta corregirlo.

## 1. Alcance

| N.º | Entorno | Subsistema | Sistema operativo |
|---|---|---|---|
| 1 | `bench_rc` | Receptor FS-R6B y calibración de canales | Windows o Ubuntu |
| 2 | `bench_drive` | Ruedas N20 y drivers TB6612FNG | Windows o Ubuntu |
| 3 | `bench_esc` | ESC: calibración de rango y prueba por consola | Windows o Ubuntu |
| 4 | `bench_thrust` | Turbinas con las perillas del radio | Windows o Ubuntu |
| 5 | `bench_i2c` | Bus I2C (líneas y dispositivos) | Windows o Ubuntu |
| 6 | `bench_imu` | IMU: lectura, inclinación y superficie | Windows o Ubuntu |
| 7 | `bench_imu_driver` | Driver `Mpu6050` del firmware | Windows o Ubuntu |
| 8 | `bench_tof` | VL53L0X y corrección de offset | Windows o Ubuntu |
| 9 | `bench_ros` | Telemetría micro-ROS por WiFi | Ubuntu |
| 10 | `cam` | ESP32-CAM y puente a ROS 2 | Windows; puente en Ubuntu |
| 11 | `risky_manual` / `risky` | Firmware completo | Windows / Ubuntu |

## 2. Requisitos

**Hardware**
- Robot con la PCB, la batería cargada y SW1 accesible.
- Radio FlySky encendido, con el receptor FS-R6B conectado a GPIO34 (CH1), GPIO35 (CH2), GPIO32 (CH5) y GPIO33 (CH6).
- Un soporte para dejar las ruedas en el aire.
- Cable USB para la ESP32 y la placa base ESP32-CAM-MB para la cámara.

**Software**
- PlatformIO.
- En Ubuntu 22.04: ROS 2 Humble y el agente micro-ROS en `~/microros_ws`.
- `include/secrets.h`, creado a partir de `include/secrets.example.h`, con la red WiFi y la IP del PC.

**Comando base.** Todos los bancos se graban y se abren con:

```
pio run -e <entorno> -t upload -t monitor
```

## 3. Reglas generales

1. **Hélices:** los bancos 3 y 4 se hacen **sin hélices**. Se montan solo para el firmware completo.
2. **Actuadores apagados:** los bancos de sensores (5 a 9) llaman a `parkActuators()` al arrancar. Ese llamado deja en bajo los pines de ruedas y ESC, para que nada se mueva mientras se prueba otra cosa.
3. **Configuración:** cualquier ajuste de pines, calibración del radio u offset del ToF se hace solo en `include/config.h`. Después del cambio, se repite el banco afectado.
4. **Windows y Linux:** al pasar el proyecto de un sistema a otro no se copia la carpeta `.pio`. Las librerías descargadas en Windows traen saltos de línea que rompen la compilación de micro-ROS en Linux.
5. **Editor:** antes de que otra herramienta modifique un archivo, se cierra su pestaña en VS Code. Así el autoguardado no sobrescribe el cambio.
6. **Registro:** cada banco se anota en la tabla de la sección 6 como **aprobado** o **no aprobado**, con observaciones.

## 4. Procedimiento por banco

### 4.1 `bench_rc`: receptor

**Objetivo:** verificar que llegan los cuatro canales y obtener los valores de calibración.

1. Graba el banco con el radio encendido.
2. El monitor muestra, cada 100 ms, el ancho de pulso de CH1, CH2, CH5 y CH6, y entre corchetes el mínimo y el máximo vistos.
3. Lleva cada palanca y cada perilla a sus dos extremos y déjalas en el centro.
4. Copia en `rc::kCh1`, `kCh2`, `kCh5` y `kCh6` de `config.h` los valores `{mínimo, centro, máximo}`.
5. Apaga el radio.

**Criterio de aprobación**
- Los cuatro canales marcan entre unos 1000 y 2000 µs y siguen al mando sin saltos.
- El mínimo, el centro y el máximo de cada canal son distintos entre sí. Si dos son iguales, el cálculo divide por cero.
- Al apagar el radio, los cuatro canales muestran `----`.

**Si falla:** revisa el cableado del canal sin señal. Si aparecen saltos aislados, revisa masa y ruido: el firmware ya filtra con una mediana de 3 pulsos.

### 4.2 `bench_drive`: ruedas

**Objetivo:** verificar el sentido de giro, la mezcla avance/giro y el freno sin señal.

1. Pon el robot sobre el soporte, con las ruedas en el aire.
2. Con la palanca derecha (CH2 avance, CH1 giro), prueba: adelante, atrás, giro a la izquierda y giro a la derecha.
3. El monitor muestra el avance y el giro pedidos y la salida de cada lado, de −1 a 1.
4. Apaga el radio con las ruedas girando.

**Criterio de aprobación**
- Con la palanca centrada, las ruedas están quietas y la salida es 0.
- Adelante: los dos lados giran hacia adelante. Atrás: los dos hacia atrás.
- Giro a la derecha: el lado izquierdo avanza más que el derecho. El giro a la izquierda es el caso opuesto.
- Al apagar el radio aparece `sin señal RC: freno` y las ruedas se detienen.

**Si falla**
- Si las ruedas se mueven con la palanca centrada, recalibra el centro en el banco 4.1.
- Si un lado entero gira al revés, cambia `drive::kLeftInverted` o `kRightInverted`.
- Si un solo motor gira al revés que los otros de su lado, intercambia sus dos cables en la salida del driver.
- Si un lado no invierte el sentido, revisa la señal IN2 de ese lado (GPIO18 o GPIO16).

### 4.3 `bench_esc`: calibración y prueba de los ESC

**Objetivo:** calibrar el rango de 1000 a 2000 µs en los dos ESC y comprobar cada turbina por separado.

1. Sin hélices y con SW1 apagado, graba el banco.
2. Escribe `max` y enciende SW1. Espera los pitidos de calibración y escribe `min`.
3. Prueba cada turbina con `1 30` y luego `2 30` (ESC1 o ESC2 al 30 %), y las dos juntas con `a 30`. Cualquier otro texto las apaga.

**Criterio de aprobación**
- Los dos ESC terminan la secuencia de calibración.
- `1` mueve solo la turbina delantera (GPIO25) y `2` solo la trasera (GPIO26).
- Las dos arrancan suave y responden a cambios de porcentaje.

**Observación conocida:** ESC1 gira algo más lento que ESC2 al mismo porcentaje.

### 4.4 `bench_thrust`: turbinas con el radio

**Objetivo:** verificar el control de las turbinas con las perillas y sus dos reglas de seguridad.

1. Sin hélices, graba el banco con el radio encendido y las perillas arriba.
2. Comprueba que el estado diga `esperando ambas perillas en 0 para armar` y que las turbinas no arranquen.
3. Baja las dos perillas a cero. El estado debe cambiar a `armado`.
4. Sube Knob A (delantera) y Knob B (trasera) por separado.
5. Con una turbina girando, apaga el radio.

**Criterio de aprobación**
- No arranca hasta ver las dos perillas en cero (armado seguro).
- Knob A controla solo la delantera y Knob B solo la trasera.
- Por debajo del 5 % de la perilla, la turbina queda apagada.
- Sin señal, aparece `SIN SEÑAL RC: mantiene la última salida` y la turbina sigue igual. Es intencional: en la pared, apagar las turbinas haría caer el robot.

### 4.5 `bench_i2c`: bus de sensores

**Objetivo:** comprobar el estado eléctrico del bus y que respondan la IMU y el ToF.

1. Graba el banco.
2. Revisa la primera línea: `Líneas en reposo: SDA ..., SCL ...`.
3. Observa varios escaneos seguidos.

**Criterio de aprobación**
- En cada escaneo aparecen `0x29 VL53L0X` y `0x68 MPU-6050`.
- No hay escaneos que fallen de forma intermitente.

**Si falla**
- Si alguna línea en reposo está en `BAJO`, los pull-ups son débiles o algo está en corto. El bus funciona a 100 kHz (`kI2cHz`) por esta razón.
- La solución definitiva es poner pull-ups de 4,7 kΩ y alimentar el módulo de la IMU a 5 V.

### 4.6 `bench_imu`: inclinación y superficie

**Objetivo:** verificar la lectura de la IMU y la clasificación de superficie que usa el firmware.

1. Graba el banco con el robot quieto. Debe mostrar `WHO_AM_I = 0x70` (MPU-6500 compatible).
2. El modo por defecto (`SUP`) imprime la inclinación, la superficie y el módulo de la aceleración.
3. Coloca el robot horizontal, luego vertical y luego invertido.
4. Si hace falta, usa los comandos `DATOS`, `CAL`, `VIB`, `INFO` y `AYUDA`.

**Criterio de aprobación**
- Con el robot quieto, |a| está cerca de 1,000 g.
- Horizontal marca una inclinación cercana a 0° y `suelo`. Vertical, cerca de 90° y `pared`. Invertido, cerca de 180° y `techo`.
- La lectura no se congela ni vuelve a ceros.

**Observación:** si en el suelo marca una inclinación distinta de 0°, revisa que la PCB esté montada nivelada.

### 4.7 `bench_imu_driver`: driver del firmware

**Objetivo:** confirmar que el driver `Mpu6050`, que usan `risky` y `bench_ros`, lee lo mismo que el banco 4.6.

1. Graba el banco con el robot quieto. Espera la calibración del giroscopio.
2. Compara las lecturas con las del banco 4.6 en las mismas posiciones.

**Criterio de aprobación**
- La aceleración está en g y suma alrededor de 1 g en reposo.
- El giroscopio está en rad/s y cerca de 0 en reposo.
- Los valores coinciden con los del banco 4.6.

### 4.8 `bench_tof`: distancia y offset

**Objetivo:** verificar el VL53L0X y dejar corregido su offset.

1. Pon el robot en el suelo, sobre el mismo tipo de superficie que se va a inspeccionar.
2. Mide con regla la distancia real desde la cara del sensor hasta el suelo.
3. Graba el banco. Cada 0,5 s aparece una línea con el promedio `corregida`, el promedio `cruda` y el número de medidas.
4. Calcula `tof::kOffsetMm = distancia real − cruda` y ponlo en `config.h`.
5. Graba de nuevo el banco.

**Criterio de aprobación**
- El sensor entrega unas 20 medidas por segundo.
- Después del ajuste, `corregida` coincide con la distancia real (diferencia de unos pocos milímetros).
- Al tapar o retirar la superficie, la distancia cambia o aparece `sin superficie a la vista`.

**Observación:** el offset se aplica dentro de la librería `Tof`. Por eso también lo usan `risky` y `bench_ros`.

### 4.9 `bench_ros`: telemetría micro-ROS (Ubuntu)

**Objetivo:** verificar la publicación de la IMU y el ToF en ROS 2 por WiFi, y la reconexión con el agente.

1. Revisa la IP del PC con `ip -4 addr` y ponla en `kAgentIp` de `secrets.h`. El PC y el robot deben estar en la misma red.
2. Inicia el agente:
   ```
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
   ```
3. Graba el banco y espera `Agente conectado, publicando.`.
4. En otra terminal:
   ```
   ros2 topic list | grep risky
   ros2 topic hz /risky/imu/data_raw
   ros2 topic echo /risky/tof/range --field range
   ```
5. Detén el agente con Ctrl+C y vuelve a lanzarlo.

**Criterio de aprobación**
- Aparecen `/risky/imu/data_raw` y `/risky/tof/range`.
- La IMU publica a unos 20 Hz.
- La distancia cambia al acercar y alejar un objeto, y marca `inf` cuando no hay superficie.
- Sin reiniciar el ESP32, se reconecta solo cuando vuelve el agente.

**Si falla**
- Si el ESP32 conecta al WiFi pero el agente no recibe nada, revisa `kAgentIp` y el firewall (`sudo ufw allow 8888/udp`). Confirma también que el PC sigue conectado a la red.
- Si hay errores de compilación con `\r` o `"." no se reconoce`, borra `.pio` y compila en Ubuntu.

### 4.10 `cam`: ESP32-CAM

**Objetivo:** verificar el video MJPEG y su publicación en ROS 2.

1. Graba la cámara con la placa base: `pio run -e cam -t upload -t monitor`. Debe aparecer `Cámara lista.` y la IP.
2. Desde un navegador en la misma red, abre `http://<IP-cam>/`. Comprueba el video y el control del flash.
3. Cierra el navegador. En Ubuntu, con el workspace compilado:
   ```
   ros2 run risky_tools camera_bridge --ros-args -p url:=http://<IP-cam>:81/stream
   ros2 topic hz /risky/camera/image/compressed
   ros2 run rqt_image_view rqt_image_view
   ```

**Criterio de aprobación**
- El video se ve en el navegador y el flash responde.
- `camera_bridge` muestra `Recibiendo video de ...`.
- La imagen se ve en `rqt_image_view`.

**Si falla**
- `Camera probe failed`: reasienta el flex del sensor y revisa la alimentación.
- `timed out`: confirma que ningún otro cliente esté viendo el video, porque la cámara atiende uno a la vez, y que el PC esté en la misma red.
- Si el video va a tirones, el límite es el enlace WiFi. Desactiva el ahorro de energía del WiFi del PC (`sudo iw dev wlp3s0 set power_save off`) o usa un router en lugar del hotspot.

### 4.11 Firmware completo

**Objetivo:** verificar el sistema integrado y que el control no dependa de la telemetría.

1. **En Windows:** graba `risky_manual` y repite en el suelo las pruebas de los bancos 4.2 y 4.4. Esta vez sí con hélices y con el robot sujeto.
2. **En Ubuntu:** con el agente corriendo, graba `risky`. Deja el robot quieto unos 2 s al encender, mientras se calibra el giroscopio.
3. Verifica los cuatro tópicos:
   ```
   ros2 topic echo /risky/cmd_vel
   ros2 topic echo /risky/thrust
   ros2 topic hz /risky/imu/data_raw
   ros2 topic echo /risky/tof/range --field range
   ```
4. **Independencia del control:** mientras manejas con el radio, detén el agente y después apaga la red WiFi.
5. Con todo corriendo (agente, robot, cámara y `camera_bridge`), revisa la arquitectura con `ros2 run rqt_graph rqt_graph`.

**Criterio de aprobación**
- El manejo por radio es igual en `risky_manual` y en `risky`.
- `/risky/cmd_vel` cambia con la palanca y `/risky/thrust` con las perillas.
- En el paso 4, las ruedas y las turbinas responden igual sin agente y sin WiFi. Al volver la red y el agente, la telemetría se reconecta sola.
- `rqt_graph` muestra el nodo del robot con sus cuatro tópicos y `camera_bridge` con la imagen.

## 5. Problemas conocidos

| Síntoma | Causa | Solución |
|---|---|---|
| Canal con calibración `{x, y, y}` | Dos valores iguales: división por cero | Recalibrar con `bench_rc` |
| Ruedas que avanzan con la palanca centrada | Centro o banda muerta mal calibrados | Recalibrar el centro en `config.h` |
| IMU en ceros o inclinación fija | Bus I2C débil | Revisar con `bench_i2c`; pull-ups de 4,7 kΩ |
| Compilación de micro-ROS con errores de `\r` | `.pio` copiada desde Windows | Borrar `.pio` y compilar en Ubuntu |
| `"." no se reconoce como un comando` | Entorno con micro-ROS compilado en Windows | Usar `risky_manual` en Windows o compilar en Ubuntu |
| `PermissionError: 'log'` en colcon | Carpeta copiada como solo lectura | `chmod -R u+w ~/Risky` |
| Cambios que desaparecen | VS Code guardó una pestaña vieja | Cerrar la pestaña sin guardar y volver a abrir |

## 6. Registro de verificación

| N.º | Banco | Fecha | Resultado | Observaciones |
|---|---|---|---|---|
| 1 | `bench_rc` | | | |
| 2 | `bench_drive` | | | |
| 3 | `bench_esc` | | | |
| 4 | `bench_thrust` | | | |
| 5 | `bench_i2c` | | | |
| 6 | `bench_imu` | | | |
| 7 | `bench_imu_driver` | | | |
| 8 | `bench_tof` | | | Offset: ___ mm |
| 9 | `bench_ros` | | | |
| 10 | `cam` | | | |
| 11 | Firmware completo | | | |
