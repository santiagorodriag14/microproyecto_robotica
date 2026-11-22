"""Gemelo digital: estima la pose del robot real y la lleva a RViz (TF, /risky/odom) y a Gazebo.

Orientación: filtro complementario de Mahony. El giroscopio da el giro y el acelerómetro corrige
alabeo y cabeceo cuando solo mide gravedad. El rumbo no tiene referencia (no hay magnetómetro):
se integra el giroscopio y se reestima su sesgo cada vez que el robot está quieto.

Posición: integrar dos veces la aceleración del MPU-6050 deriva metros en segundos, así que el
robot avanza por su eje x con la velocidad de /risky/cmd_vel. Como ese eje sale de la IMU, al
inclinarse en la pared el avance se vuelve subida y en el techo vuelve a ser horizontal.

Contacto: la IMU va en el segmento trasero. Ese segmento se apoya en la superficie que tiene debajo
y ninguna rueda atraviesa el suelo, la pared ni el techo de risky_course.sdf. La articulación
delantera toma el menor ángulo que deja sus ruedas sobre la superficie que tiene enfrente.
"""
import math

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

GRAVITY = 9.80665
WHEEL_RADIUS = 0.0175
TRACK_WIDTH = 0.255
LEFT_WHEELS = ['Rueda_l1_Joint', 'Rueda_l2_Joint', 'Rueda_l3_Joint', 'Rueda_l4_Joint']
RIGHT_WHEELS = ['Rueda_r1_Joint', 'Rueda_r2_Joint', 'Rueda_r3_Joint', 'Rueda_r4_Joint']
FRONT_JOINT = 'Chasis_frontal_Joint'
PROPELLERS = ['Aspa_front_Joint', 'Aspa_back_Joint']
# Centros de rueda en base_link, del URDF. Las del eje de la articulación no dependen de su ángulo.
REAR_WHEELS = np.array([[-0.2322, -0.1166, -0.005], [-0.2322, 0.1170, -0.005],
                        [-0.0312, -0.1380, -0.005], [-0.0312, 0.1383, -0.005],
                        [0.0418, -0.1380, -0.005], [0.0418, 0.1382, -0.005]])
FRONT_PIVOT = np.array([0.0418, -0.0019, -0.005])
FRONT_LENGTH = 0.238
FRONT_WHEEL_Y = np.array([-0.1148, 0.1188])
FRONT_ANGLES = np.radians(np.arange(0.0, 90.5, 0.5))
# Por debajo de esta inclinación se toma como ruido de la IMU. Más lejos que CORNER_REACH no se
# salta a la esquina: sería un error de la cuenta muerta, no un apoyo real.
CORNER_TILT = math.radians(5.0)
CORNER_REACH = 0.15
# Apoyado en una pared el robot solo puede estar perpendicular a ella: su cara inferior mira a la
# pared. Desde esta inclinación ya hay componente horizontal suficiente para corregir el rumbo.
WALL_ALIGN_TILT = math.radians(20.0)
WALL_ALIGN_GAIN = 0.2


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


def rotation_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TwinEstimator(Node):

    def __init__(self):
        super().__init__('twin_estimator')
        p = self.declare_parameter
        # Corrige el cmd_vel del ESP32, que sale de kMaxWheelSpeedMps (estimado, no medido).
        self.speed_scale = p('speed_scale', 1.0).value
        # Más alto: el acelerómetro corrige más rápido pero deja pasar más vibración.
        self.kp = p('kp', 1.0).value
        # Solo se confía en el acelerómetro si |a| está a menos de esta fracción de g.
        self.accel_gate = p('accel_gate', 0.15).value
        # El ESP32 publica el comando a 20 Hz aunque la palanca esté al centro: si deja de llegar es un
        # corte de WiFi y el robot sigue con el último. Mismo límite que los huecos de la IMU.
        # Se han medido cortes de 2,2 s en mitad de un giro: descartarlos dejaba el rumbo 50° corto.
        self.max_gap = p('max_gap_s', 5.0).value
        self.cmd_timeout = p('cmd_timeout_s', self.max_gap).value
        # Toma como plano lo que mide la IMU en los primeros segundos quieto en el suelo y descuenta
        # la inclinación del chip dentro del robot. Desactivar si el piso del arranque no está a nivel.
        self.level_at_start = p('level_at_start', True).value
        self.level_samples = p('level_samples', 40).value
        # Superficies del circuito como planos (normal hacia el aire, n·x >= d). Copiadas de
        # risky_course.sdf: si se mueve la pared o el techo, cambiar estos parámetros.
        wall_x = p('wall_x', 0.80).value
        ceiling_z = p('ceiling_z', 1.0).value
        self.planes = [(np.array([0.0, 0.0, 1.0]), 0.0),
                       (np.array([-1.0, 0.0, 0.0]), -wall_x),
                       (np.array([0.0, 0.0, -1.0]), -ceiling_z)]
        self.start = np.array([p('x0', 0.0).value, p('y0', 0.0).value, WHEEL_RADIUS + 0.005])
        self.start_yaw = p('yaw0', 0.0).value
        self.model = p('gz_model', 'risky').value
        self.gz_period = 1.0 / p('gz_rate_hz', 20.0).value

        self.q = None
        self.position = self.start.copy()
        self.gyro_bias = np.zeros(3)
        self.still_since = None
        self.last_imu_t = None
        self.last_gyro = None
        self.cmd = Twist()
        self.cmd_t = -math.inf
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.front_angle = 0.0
        self.last_gz_t = -math.inf
        self.gz_pending = None

        self.odom_pub = self.create_publisher(Odometry, '/risky/odom', 10)
        self.joints_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.gz_front = self.create_publisher(Float64, '/sim/twin/front_joint', 10)
        self.tf = TransformBroadcaster(self)
        # Montaje del chip respecto a base_link, tal como lo declara el URDF.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.imu_mount = None
        self.level = np.eye(3)
        self.level_buffer = []
        world = p('gz_world', 'risky_course').value
        self.gz = self.create_client(SetEntityPose, f'/world/{world}/set_pose')
        self.create_subscription(Imu, '/risky/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, '/risky/cmd_vel', self.on_cmd, qos_profile_sensor_data)
        self.create_service(Trigger, '/risky/twin/reset', self.on_reset)

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_t = self.get_clock().now().nanoseconds * 1e-9

    def on_reset(self, request, response):
        self.q = None
        self.last_gyro = None
        self.level_buffer = []
        self.position = self.start.copy()
        self.left_angle = self.right_angle = self.front_angle = 0.0
        response.success = True
        response.message = 'Pose reiniciada al punto de partida'
        return response

    def commanded_speed(self):
        if self.get_clock().now().nanoseconds * 1e-9 - self.cmd_t > self.cmd_timeout:
            return 0.0, 0.0
        return self.cmd.linear.x * self.speed_scale, self.cmd.angular.z

    def collect_level(self, gyro, accel):
        """Junta muestras quietas; al completarlas fija la rotación que deja su gravedad en +z."""
        moving = np.linalg.norm(gyro) > 0.05
        if moving or abs(np.linalg.norm(accel) - GRAVITY) > self.accel_gate * GRAVITY:
            self.level_buffer = []
            self.get_logger().info('Nivelando: deje el robot quieto en el suelo', throttle_duration_sec=3.0)
            return False
        self.level_buffer.append(accel)
        if len(self.level_buffer) < self.level_samples:
            return False
        up = np.mean(self.level_buffer, axis=0)
        up /= np.linalg.norm(up)
        axis = np.cross(up, [0.0, 0.0, 1.0])
        angle = math.atan2(np.linalg.norm(axis), up[2])
        self.level = np.eye(3)
        if np.linalg.norm(axis) > 1e-9:
            k = axis / np.linalg.norm(axis)
            skew = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
            self.level = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew
        self.get_logger().info(f'Nivelado: chip inclinado {math.degrees(angle):.1f}° respecto al piso')
        return True

    def initial_attitude(self, accel):
        ax, ay, az = accel
        return quat_from_rpy(math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az)), self.start_yaw)

    def update_gyro_bias(self, gyro, t, speed, turn):
        # Quieto: sin comando y sin giro apreciable durante 1 s. Lo que mida el giroscopio es sesgo.
        if speed == 0.0 and turn == 0.0 and np.linalg.norm(gyro - self.gyro_bias) < 0.05:
            if self.still_since is None:
                self.still_since = t
            elif t - self.still_since > 1.0:
                self.gyro_bias += 0.02 * (gyro - self.gyro_bias)
        else:
            self.still_since = None

    def update_attitude(self, gyro, accel, dt):
        omega = gyro - self.gyro_bias
        norm = np.linalg.norm(accel)
        if abs(norm - GRAVITY) < self.accel_gate * GRAVITY:
            # Gravedad que predice la orientación actual, vista desde el robot (tercera fila de R).
            predicted_up = rotation_matrix(self.q)[2]
            omega = omega + self.kp * np.cross(accel / norm, predicted_up)
        half_angle = np.linalg.norm(omega) * dt / 2
        if half_angle > 0.0:
            axis = omega / np.linalg.norm(omega)
            self.q = quat_multiply(self.q, np.concatenate(([math.cos(half_angle)],
                                                           math.sin(half_angle) * axis)))
            self.q /= np.linalg.norm(self.q)

    def lookup_imu_mount(self, frame):
        try:
            r = self.tf_buffer.lookup_transform('base_link', frame, rclpy.time.Time()).transform.rotation
        except TransformException:
            self.get_logger().warn(f'Esperando la TF base_link -> {frame} de robot_state_publisher',
                                   throttle_duration_sec=5.0)
            return None
        return rotation_matrix((r.w, r.x, r.y, r.z))

    def on_imu(self, msg):
        if self.imu_mount is None:
            self.imu_mount = self.lookup_imu_mount(msg.header.frame_id or 'imu_link')
            if self.imu_mount is None:
                return
        gyro = self.imu_mount @ np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                                          msg.angular_velocity.z])
        accel = self.imu_mount @ np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                                           msg.linear_acceleration.z])
        t = stamp_seconds(msg.header.stamp)
        if self.q is None:
            if self.level_at_start and not self.collect_level(gyro, accel):
                return
            self.q = self.initial_attitude(self.level @ accel)
            self.last_imu_t = t
            self.last_gyro = self.level @ gyro
            return
        gyro, accel = self.level @ gyro, self.level @ accel
        dt = t - self.last_imu_t
        if dt <= 0.0:
            return  # repetido o desordenado
        self.last_imu_t = t
        # Por WiFi se pierden ráfagas de hasta ~1 s, justo en los giros. Se integra el promedio de
        # la lectura anterior y la actual (trapecio): con giro parejo recupera casi todo el tramo.
        mean_gyro = (self.last_gyro + gyro) / 2
        self.last_gyro = gyro
        if dt > self.max_gap:
            self.get_logger().warn(f'Sin IMU durante {dt:.1f} s: ese tramo no se integra')
            return

        speed, turn = self.commanded_speed()
        self.update_gyro_bias(gyro, t, speed, turn)
        self.update_attitude(mean_gyro, accel, dt)
        self.align_to_wall()
        rotation = rotation_matrix(self.q)
        self.position += rotation[:, 0] * speed * dt
        self.settle_rear(rotation)
        self.front_angle = self.fit_front(rotation)

        yaw_rate = gyro[2] - self.gyro_bias[2]
        self.left_angle += (speed - yaw_rate * TRACK_WIDTH / 2) / WHEEL_RADIUS * dt
        self.right_angle += (speed + yaw_rate * TRACK_WIDTH / 2) / WHEEL_RADIUS * dt
        self.publish(msg.header.stamp, speed, gyro - self.gyro_bias)

    def publish(self, stamp, speed, omega):
        x, y, z = self.position
        qw, qx, qy, qz = self.q

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        r = tf.transform.rotation
        r.w, r.x, r.y, r.z = qw, qx, qy, qz
        self.tf.sendTransform(tf)

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
        odom.pose.pose.orientation = r
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = LEFT_WHEELS + RIGHT_WHEELS + [FRONT_JOINT] + PROPELLERS
        joints.position = ([self.left_angle] * 4 + [self.right_angle] * 4 + [self.front_angle]
                           + [0.0] * len(PROPELLERS))
        self.joints_pub.publish(joints)
        self.gz_front.publish(Float64(data=self.front_angle))

        self.move_gazebo_model(odom.pose.pose)

    def gaps(self, points):
        """Holgura de cada rueda a cada plano, en metros: cero es justo apoyada."""
        return np.array([points @ n - d - WHEEL_RADIUS for n, d in self.planes])

    def lean_on(self, plane, rotation):
        gap = self.gaps(self.position + REAR_WHEELS @ rotation.T)[plane].min()
        self.position -= self.planes[plane][0] * gap

    def settle_rear(self, rotation):
        # El empuje aprieta el robot contra la superficie más alineada con su cara inferior.
        up = rotation[:, 2]
        support, corner = sorted(range(len(self.planes)), key=lambda i: -(self.planes[i][0] @ up))[:2]
        self.lean_on(support, rotation)
        # Inclinado respecto a su superficie, el segmento solo se sostiene apoyado en la esquina: un
        # extremo en cada plano. Pasa al subir de frente (la bisagra contra la pared) y de reversa
        # (la cola contra la pared y la bisagra en el suelo).
        tilt = math.acos(min(1.0, self.planes[support][0] @ up))
        if tilt > CORNER_TILT and self.planes[corner][0] @ up > 0.0:
            gap = self.gaps(self.position + REAR_WHEELS @ rotation.T)[corner].min()
            if gap < CORNER_REACH:
                self.lean_on(corner, rotation)
        # Dos pasadas bastan para la esquina, donde una corrección empuja contra el otro plano.
        for _ in range(2):
            gaps = self.gaps(self.position + REAR_WHEELS @ rotation.T)
            for (normal, _d), plane_gaps in zip(self.planes, gaps):
                self.position += normal * max(0.0, -plane_gaps.min())

    def align_to_wall(self):
        """Corrige el rumbo, que solo da el giroscopio, con el contacto contra una pared vertical."""
        rotation = rotation_matrix(self.q)
        up = rotation[:, 2]
        horizontal = math.hypot(up[0], up[1])
        if horizontal < math.sin(WALL_ALIGN_TILT):
            return
        gaps = self.gaps(self.position + REAR_WHEELS @ rotation.T)
        for (normal, _d), plane_gaps in zip(self.planes, gaps):
            if abs(normal[2]) > 1e-6 or plane_gaps.min() > CORNER_REACH:
                continue
            error = math.atan2(normal[1], normal[0]) - math.atan2(up[1], up[0])
            error = math.atan2(math.sin(error), math.cos(error))
            # Un giro alrededor del z del mundo no cambia la inclinación que mide la IMU.
            half = WALL_ALIGN_GAIN * error / 2
            self.q = quat_multiply(np.array([math.cos(half), 0.0, 0.0, math.sin(half)]), self.q)
            self.q /= np.linalg.norm(self.q)
            return

    def fit_front(self, rotation):
        pivot = self.position + rotation @ FRONT_PIVOT
        for angle in FRONT_ANGLES:
            tips = np.array([[FRONT_LENGTH * math.cos(angle), y, FRONT_LENGTH * math.sin(angle)]
                             for y in FRONT_WHEEL_Y])
            if self.gaps(pivot + tips @ rotation.T).min() >= -1e-4:
                return float(angle)
        return float(FRONT_ANGLES[-1])

    def move_gazebo_model(self, pose):
        now = self.get_clock().now().nanoseconds * 1e-9
        busy = self.gz_pending is not None and not self.gz_pending.done()
        if busy or now - self.last_gz_t < self.gz_period or not self.gz.service_is_ready():
            return
        request = SetEntityPose.Request()
        request.entity.name = self.model
        request.entity.type = Entity.MODEL
        request.pose = pose
        self.gz_pending = self.gz.call_async(request)
        self.last_gz_t = now


def main():
    rclpy.init()
    node = TwinEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
