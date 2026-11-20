"""Gazebo con Risky, bridge de tópicos, adaptador de empuje y RViz.

Teleoperación en otra terminal: ros2 run risky_tools keyboard_teleop
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('risky_description')
    gazebo_share = get_package_share_directory('risky_gazebo')
    world = os.path.join(gazebo_share, 'worlds', 'risky_course.sdf')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(description_share, 'urdf', 'risky.urdf.xacro')]),
        value_type=str)
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('bridge_config',
                              default_value=os.path.join(gazebo_share, 'config', 'bridge.yaml')),
        # Empuje por turbina con nivel 1.0, en newtons. Con 2 kg y mu = 1: ~10 N alcanza para subir
        # la pared y ~20 N para pasar al techo y avanzar por él. Reemplazar por el valor medido.
        DeclareLaunchArgument('max_thrust_n', default_value='20.0'),
        # Gazebo y el bridge se descubren por multicast; en el loopback no dependen del WiFi.
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        # Gazebo resuelve package://risky_description/meshes buscando en esta carpeta.
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.dirname(description_share)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {world}'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time]),
        # Las ruedas quedan 2,25 cm bajo base_link; se suelta un poco más arriba.
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'risky', '-z', '0.03']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': LaunchConfiguration('bridge_config')}, sim_time]),
        Node(package='risky_tools', executable='thrust_adapter',
             parameters=[{'max_thrust_n': ParameterValue(LaunchConfiguration('max_thrust_n'),
                                                         value_type=float)}, sim_time]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(description_share, 'rviz', 'risky.rviz')],
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
