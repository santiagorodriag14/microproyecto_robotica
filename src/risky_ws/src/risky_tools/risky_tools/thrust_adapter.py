"""Entrada de la simulación: /risky/thrust (nivel 0-1 por turbina) en newtons y /risky/cmd_vel reenviado.

El ESP32 publica best effort y el bridge de Gazebo solo se suscribe reliable, así que sin este
reenvío los comandos del robot real nunca llegan a la simulación.
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Float64


class ThrustAdapter(Node):

    def __init__(self):
        super().__init__('thrust_adapter')
        # Empuje estático con el ESC al 100 %; reemplazar por el medido en banco.
        self.max_thrust_n = self.declare_parameter('max_thrust_n', 7.0).value
        # El empuje estático crece más o menos con el cuadrado de las RPM.
        self.exponent = self.declare_parameter('exponent', 2.0).value
        self.front = self.create_publisher(Float64, '/sim/thrust_front_n', 10)
        self.rear = self.create_publisher(Float64, '/sim/thrust_rear_n', 10)
        self.cmd_vel = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/risky/thrust', self.on_thrust,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/risky/cmd_vel', self.cmd_vel.publish, qos_profile_sensor_data)

    def to_newtons(self, level):
        return self.max_thrust_n * min(max(level, 0.0), 1.0) ** self.exponent

    def on_thrust(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn('/risky/thrust debe traer [delantera, trasera]',
                                   throttle_duration_sec=5.0)
            return
        self.front.publish(Float64(data=self.to_newtons(msg.data[0])))
        self.rear.publish(Float64(data=self.to_newtons(msg.data[1])))


def main():
    rclpy.init()
    node = ThrustAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
