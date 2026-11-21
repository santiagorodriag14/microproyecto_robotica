"""Panel en vivo de Risky: video de la ESP32-CAM, IMU y ToF."""
from collections import deque
import math
import signal
import sys
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
import rclpy.time
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, Range
from tf2_ros import Buffer, TransformException, TransformListener

from risky_tools.twin_estimator import rotation_matrix

WINDOW_S = 10.0
STALE_S = 1.0
RAW_FORMATS = {
    'rgb8': QtGui.QImage.Format_RGB888,
    'bgr8': QtGui.QImage.Format_BGR888,
    'mono8': QtGui.QImage.Format_Grayscale8,
}

BG = '#14171c'
PANEL = '#1c2027'
GRID = '#2c323c'
TEXT = '#d7dce3'
MUTED = '#8a93a0'
X_COLOR = '#e5604d'
Y_COLOR = '#5cb85c'
Z_COLOR = '#4a90d9'
OK_COLOR = '#5cb85c'
BAD_COLOR = '#e5604d'


class Stream:
    """Muestras recientes de un tópico y su frecuencia de llegada."""

    def __init__(self, channels):
        self.samples = deque()
        self.arrivals = deque()
        self.channels = channels

    def add(self, values):
        now = time.monotonic()
        self.samples.append((now, values))
        self.arrivals.append(now)
        while self.samples and now - self.samples[0][0] > WINDOW_S:
            self.samples.popleft()
        while now - self.arrivals[0] > 2.0:
            self.arrivals.popleft()

    def hz(self, now):
        recent = [t for t in self.arrivals if now - t <= 2.0]
        return len(recent) / 2.0

    def age(self, now):
        return now - self.arrivals[-1] if self.arrivals else math.inf


class DashboardNode(Node):

    def __init__(self):
        super().__init__('risky_dashboard')
        self.lock = threading.Lock()
        self.accel = Stream(3)
        self.gyro = Stream(3)
        self.tof = Stream(1)
        self.camera = Stream(0)
        self.last_range = None
        self.max_range = None
        self.frame = None
        self.frame_seq = 0
        self.imu_frame = 'imu_link'
        self.imu_mount = None
        # Las gráficas muestran los ejes del chip. La inclinación se da en los del robot, que
        # necesitan el montaje del URDF (lo publica robot_state_publisher).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Imu, '/risky/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Range, '/risky/tof/range', self.on_range, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, '/risky/camera/image/compressed',
                                 self.on_jpeg, qos_profile_sensor_data)
        # La cámara de Gazebo publica sin comprimir.
        self.create_subscription(Image, '/risky/camera/image_raw', self.on_raw, qos_profile_sensor_data)

    def on_imu(self, msg):
        a, w = msg.linear_acceleration, msg.angular_velocity
        with self.lock:
            self.imu_frame = msg.header.frame_id or self.imu_frame
            self.accel.add((a.x, a.y, a.z))
            self.gyro.add(tuple(math.degrees(v) for v in (w.x, w.y, w.z)))

    def to_robot_axes(self, vector):
        """Vector del chip en ejes de base_link, o None mientras no haya TF."""
        if self.imu_mount is None:
            try:
                r = self.tf_buffer.lookup_transform('base_link', self.imu_frame,
                                                    rclpy.time.Time()).transform.rotation
            except TransformException:
                return None
            self.imu_mount = rotation_matrix((r.w, r.x, r.y, r.z))
        return self.imu_mount @ vector

    def on_range(self, msg):
        with self.lock:
            # El firmware publica inf cuando el VL53L0X no ve superficie.
            value = msg.range if math.isfinite(msg.range) else None
            self.last_range = value
            self.max_range = msg.max_range if msg.max_range > 0 else None
            self.tof.add((value,))

    def on_jpeg(self, msg):
        self.store_frame(('jpeg', bytes(msg.data)))

    def on_raw(self, msg):
        if msg.encoding in RAW_FORMATS:
            self.store_frame((msg.encoding, bytes(msg.data), msg.width, msg.height, msg.step))
        else:
            self.get_logger().warning(f'Codificación no soportada: {msg.encoding}',
                                      throttle_duration_sec=10)

    def store_frame(self, frame):
        with self.lock:
            self.frame = frame
            self.frame_seq += 1
            self.camera.add(())


class Plot(QtWidgets.QWidget):

    def __init__(self, title, unit, labels, colors, min_span):
        super().__init__()
        self.title, self.unit = title, unit
        self.labels, self.colors = labels, colors
        self.min_span = min_span
        self.samples = []
        self.setMinimumHeight(150)

    def set_samples(self, samples):
        self.samples = samples
        self.update()

    def y_limits(self):
        values = [v for _, vals in self.samples for v in vals if v is not None]
        if not values:
            return -self.min_span / 2, self.min_span / 2
        lo, hi = min(values), max(values)
        if hi - lo < self.min_span:
            mid = (hi + lo) / 2
            lo, hi = mid - self.min_span / 2, mid + self.min_span / 2
        pad = (hi - lo) * 0.08
        return lo - pad, hi + pad

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor(PANEL))
        left, top, right, bottom = 56, 26, 10, 22
        area = QtCore.QRectF(left, top, self.width() - left - right,
                             self.height() - top - bottom)

        p.setPen(QtGui.QColor(TEXT))
        p.setFont(QtGui.QFont('Sans', 10, QtGui.QFont.Bold))
        p.drawText(10, 18, f'{self.title} ({self.unit})')

        latest = self.samples[-1][1] if self.samples else None
        p.setFont(QtGui.QFont('Monospace', 9))
        x = self.width() - 10
        for i in reversed(range(len(self.labels))):
            value = latest[i] if latest and latest[i] is not None else None
            text = f'{self.labels[i]} {value:+8.2f}' if value is not None else f'{self.labels[i]}    ---'
            w = p.fontMetrics().horizontalAdvance(text)
            x -= w
            p.setPen(QtGui.QColor(self.colors[i]))
            p.drawText(int(x), 18, text)
            x -= 14

        lo, hi = self.y_limits()
        p.setFont(QtGui.QFont('Sans', 8))
        for k in range(5):
            value = lo + (hi - lo) * k / 4
            y = area.bottom() - area.height() * k / 4
            p.setPen(QtGui.QPen(QtGui.QColor(GRID), 1))
            p.drawLine(QtCore.QPointF(area.left(), y), QtCore.QPointF(area.right(), y))
            p.setPen(QtGui.QColor(MUTED))
            p.drawText(QtCore.QRectF(0, y - 8, left - 6, 16),
                       QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, f'{round(value, 1) + 0.0:.1f}')
        p.drawText(QtCore.QRectF(area.left(), area.bottom() + 4, area.width(), 16),
                   QtCore.Qt.AlignLeft, f'-{WINDOW_S:.0f} s')
        p.drawText(QtCore.QRectF(area.left(), area.bottom() + 4, area.width(), 16),
                   QtCore.Qt.AlignRight, 'ahora')

        if len(self.samples) < 2:
            return
        now = time.monotonic()

        def point(t, v):
            px = area.right() - (now - t) / WINDOW_S * area.width()
            py = area.bottom() - (v - lo) / (hi - lo) * area.height()
            return QtCore.QPointF(px, py)

        p.setClipRect(area)
        for i, color in enumerate(self.colors):
            p.setPen(QtGui.QPen(QtGui.QColor(color), 1.6))
            path, drawing = QtGui.QPainterPath(), False
            for t, vals in self.samples:
                if vals[i] is None:
                    drawing = False
                    continue
                if drawing:
                    path.lineTo(point(t, vals[i]))
                else:
                    path.moveTo(point(t, vals[i]))
                    drawing = True
            p.drawPath(path)


class CameraView(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        self.image = None
        self.connected = False
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def set_frame(self, frame):
        if frame[0] == 'jpeg':
            image = QtGui.QImage.fromData(frame[1], 'JPG')
        else:
            encoding, data, width, height, step = frame
            # copy(): el QImage no se adueña del buffer y data se libera al salir.
            image = QtGui.QImage(data, width, height, step, RAW_FORMATS[encoding]).copy()
        if not image.isNull():
            self.image = image

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor('#000000'))
        if self.image:
            scaled = self.image.size().scaled(self.size(), QtCore.Qt.KeepAspectRatio)
            target = QtCore.QRect(QtCore.QPoint(0, 0), scaled)
            target.moveCenter(self.rect().center())
            p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
            p.drawImage(target, self.image)
        if not self.connected:
            p.fillRect(self.rect(), QtGui.QColor(0, 0, 0, 150))
            p.setPen(QtGui.QColor(TEXT))
            p.setFont(QtGui.QFont('Sans', 13))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, 'Sin video')


def big_label(size):
    label = QtWidgets.QLabel('---')
    label.setStyleSheet(f'color: {TEXT}; font-size: {size}px; font-weight: bold;')
    return label


def small_label(text=''):
    label = QtWidgets.QLabel(text)
    label.setStyleSheet(f'color: {MUTED}; font-size: 12px;')
    return label


def panel(title, *widgets):
    box = QtWidgets.QFrame()
    box.setStyleSheet(f'QFrame {{ background: {PANEL}; border-radius: 6px; }}')
    layout = QtWidgets.QVBoxLayout(box)
    layout.setContentsMargins(12, 8, 12, 10)
    header = QtWidgets.QLabel(title)
    header.setStyleSheet(f'color: {TEXT}; font-size: 13px; font-weight: bold;')
    layout.addWidget(header)
    for widget in widgets:
        layout.addWidget(widget)
    return box


class Dashboard(QtWidgets.QMainWindow):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.frame_seq = 0
        self.setWindowTitle('Risky')
        self.resize(1280, 760)
        self.setStyleSheet(f'QMainWindow {{ background: {BG}; }}')

        self.camera = CameraView()
        self.camera_info = small_label()

        self.tof_value = big_label(40)
        self.tof_info = small_label()
        self.roll = big_label(26)
        self.pitch = big_label(26)
        self.gravity = small_label()

        attitude = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(attitude)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.addWidget(small_label('Alabeo'), 0, 0)
        grid.addWidget(small_label('Cabeceo'), 0, 1)
        grid.addWidget(self.roll, 1, 0)
        grid.addWidget(self.pitch, 1, 1)
        grid.addWidget(self.gravity, 2, 0, 1, 2)

        self.accel_plot = Plot('Aceleración', 'm/s²', 'xyz', (X_COLOR, Y_COLOR, Z_COLOR), 2.0)
        self.gyro_plot = Plot('Giro', '°/s', 'xyz', (X_COLOR, Y_COLOR, Z_COLOR), 20.0)
        self.tof_plot = Plot('Distancia ToF', 'mm', 'd', (Z_COLOR,), 50.0)

        self.status = {name: small_label() for name in ('IMU', 'ToF', 'Cámara')}

        top_right = QtWidgets.QHBoxLayout()
        top_right.addWidget(panel('ToF', self.tof_value, self.tof_info), 1)
        top_right.addWidget(panel('Inclinación (del acelerómetro)', attitude), 1)

        right = QtWidgets.QVBoxLayout()
        right.addLayout(top_right)
        right.addWidget(self.accel_plot, 1)
        right.addWidget(self.gyro_plot, 1)
        right.addWidget(self.tof_plot, 1)

        left = QtWidgets.QVBoxLayout()
        left.addWidget(self.camera, 1)
        left.addWidget(self.camera_info)

        body = QtWidgets.QHBoxLayout()
        body.addLayout(left, 5)
        body.addLayout(right, 4)

        status_bar = QtWidgets.QHBoxLayout()
        for label in self.status.values():
            status_bar.addWidget(label)
        status_bar.addStretch()

        root = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 8)
        layout.addLayout(body, 1)
        layout.addLayout(status_bar)
        self.setCentralWidget(root)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.refresh)
        timer.start(33)

    def refresh(self):
        node, now = self.node, time.monotonic()
        with node.lock:
            accel = list(node.accel.samples)
            gyro = list(node.gyro.samples)
            tof = list(node.tof.samples)
            last_range, max_range = node.last_range, node.max_range
            frame = node.frame if node.frame_seq != self.frame_seq else None
            self.frame_seq = node.frame_seq
            streams = {'IMU': node.accel, 'ToF': node.tof, 'Cámara': node.camera}
            rates = {name: (s.hz(now), s.age(now)) for name, s in streams.items()}

        if frame:
            self.camera.set_frame(frame)
        self.camera.connected = rates['Cámara'][1] < STALE_S
        if self.camera.image:
            size = self.camera.image.size()
            self.camera_info.setText(
                f'{size.width()}×{size.height()}  ·  {rates["Cámara"][0]:.1f} fps')
        self.camera.update()

        self.accel_plot.set_samples(accel)
        self.gyro_plot.set_samples(gyro)
        self.tof_plot.set_samples(
            [(t, (v[0] * 1000 if v[0] is not None else None,)) for t, v in tof])

        if rates['ToF'][1] > STALE_S:
            self.tof_value.setText('---')
        elif last_range is None:
            self.tof_value.setText('sin blanco')
        else:
            self.tof_value.setText(f'{last_range * 1000:.0f} mm')
        self.tof_info.setText(f'rango máx. {max_range * 1000:.0f} mm' if max_range else '')

        if accel and rates['IMU'][1] < STALE_S:
            ax, ay, az = accel[-1][1]
            robot = self.node.to_robot_axes(accel[-1][1])
            if robot is not None:
                ax, ay, az = robot
            # Solo vale con el robot casi quieto: la aceleración propia se suma a la gravedad.
            self.roll.setText(f'{math.degrees(math.atan2(ay, az)):+.1f}°')
            self.pitch.setText(f'{math.degrees(math.atan2(-ax, math.hypot(ay, az))):+.1f}°')
            self.gravity.setText(f'|a| = {math.sqrt(ax * ax + ay * ay + az * az):.2f} m/s²')
        else:
            self.roll.setText('---')
            self.pitch.setText('---')
            self.gravity.setText('')

        for name, (hz, age) in rates.items():
            alive = age < STALE_S
            color = OK_COLOR if alive else BAD_COLOR
            state = f'{hz:.1f} Hz' if alive else 'sin datos'
            self.status[name].setText(
                f'<span style="color:{color}">●</span> {name}: {state}&nbsp;&nbsp;&nbsp;')


def main():
    rclpy.init()
    node = DashboardNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    app = QtWidgets.QApplication(sys.argv)
    window = Dashboard(node)
    window.show()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    # Python solo atiende señales cuando Qt le devuelve el control; este timer lo fuerza.
    signal_timer = QtCore.QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)
    try:
        app.exec_()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
