"""Publica el MJPEG de la ESP32-CAM como sensor_msgs/CompressedImage."""
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CameraBridge(Node):

    def __init__(self):
        super().__init__('camera_bridge')
        self.url = self.declare_parameter('url', 'http://risky-cam.local:81/stream').value
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        self.pub = self.create_publisher(
            CompressedImage, '/risky/camera/image/compressed', qos_profile_sensor_data)
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while rclpy.ok():
            try:
                # El timeout también corta lecturas: si la cámara se cuelga, se reconecta.
                with urllib.request.urlopen(self.url, timeout=5) as stream:
                    self.get_logger().info(f'Recibiendo video de {self.url}')
                    self.read_frames(stream)
            except OSError as error:
                self.get_logger().warning(
                    f'Sin video de {self.url}: {error}', throttle_duration_sec=10)
            time.sleep(2)

    def read_frames(self, stream):
        while rclpy.ok():
            length = None
            while True:
                line = stream.readline()
                if not line:
                    return
                line = line.strip()
                if line.lower().startswith(b'content-length:'):
                    length = int(line.split(b':')[1])
                elif not line and length is not None:
                    break
            msg = CompressedImage(format='jpeg', data=stream.read(length))
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
