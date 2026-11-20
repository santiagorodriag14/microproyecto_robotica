"""Emula el radio desde el teclado: publica /risky/cmd_vel y /risky/thrust igual que el ESP32."""
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

HELP = """
Ruedas:   w / x  avanzar / retroceder    a / d  girar izq. / der.    s  frenar
Turbinas: t / b  ambas + / -    y / n  delantera + / -    u / m  trasera + / -    0  apagar
Ctrl+C para salir
"""


def clamp(value, limit):
    return max(-limit, min(limit, value))


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.max_linear = self.declare_parameter('max_linear', 0.10).value
        self.max_angular = self.declare_parameter('max_angular', 1.0).value
        self.linear_step = self.declare_parameter('linear_step', 0.01).value
        self.angular_step = self.declare_parameter('angular_step', 0.1).value
        self.thrust_step = self.declare_parameter('thrust_step', 0.05).value
        self.cmd_pub = self.create_publisher(Twist, '/risky/cmd_vel', 10)
        self.thrust_pub = self.create_publisher(Float32MultiArray, '/risky/thrust', 10)
        self.linear = 0.0
        self.angular = 0.0
        self.thrust = [0.0, 0.0]  # [delantera, trasera], mismo orden que el ESP32

    def on_key(self, key):
        wheel_keys = {
            'w': (self.linear_step, 0.0), 'x': (-self.linear_step, 0.0),
            'a': (0.0, self.angular_step), 'd': (0.0, -self.angular_step),
        }
        thrust_keys = {
            't': (1, 1), 'b': (-1, -1), 'y': (1, 0), 'n': (-1, 0), 'u': (0, 1), 'm': (0, -1),
        }
        if key in wheel_keys:
            dl, da = wheel_keys[key]
            self.linear = clamp(self.linear + dl, self.max_linear)
            self.angular = clamp(self.angular + da, self.max_angular)
        elif key == 's':
            self.linear = self.angular = 0.0
        elif key in thrust_keys:
            for i, sign in enumerate(thrust_keys[key]):
                self.thrust[i] = min(max(self.thrust[i] + sign * self.thrust_step, 0.0), 1.0)
        elif key == '0':
            self.thrust = [0.0, 0.0]

    def publish(self):
        cmd = Twist()
        cmd.linear.x = self.linear
        cmd.angular.z = self.angular
        self.cmd_pub.publish(cmd)
        self.thrust_pub.publish(Float32MultiArray(data=self.thrust))
        print(f'\rv {self.linear:+.2f} m/s  w {self.angular:+.2f} rad/s  '
              f'turbinas del {self.thrust[0]:.2f} tras {self.thrust[1]:.2f}   ', end='', flush=True)


def read_key(timeout_s):
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    return sys.stdin.read(1) if ready else ''


def main():
    rclpy.init()
    node = KeyboardTeleop()
    saved = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        # Publicar aunque no haya teclas, como el ESP32 que reenvía el radio continuamente.
        while rclpy.ok():
            node.on_key(read_key(0.05))
            node.publish()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()
