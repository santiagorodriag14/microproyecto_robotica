"""Robot real y simulación juntos: agente micro-ROS (IMU y ToF), puente de la ESP32-CAM,
Gazebo con RViz y panel de datos.

El agente vive en otro workspace: antes de lanzar, source ~/microros_ws/install/setup.bash
"""
import os
import tempfile

from ament_index_python.packages import (PackageNotFoundError, get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

REAL_SENSOR_TOPICS = ('/risky/imu/', '/risky/tof/', '/risky/camera/')


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def micro_ros_agent(context):
    if not is_true(context, 'agent'):
        return []
    try:
        get_package_prefix('micro_ros_agent')
    except PackageNotFoundError:
        return [LogInfo(msg='No se encontró micro_ros_agent: haga '
                            'source ~/microros_ws/install/setup.bash. Sin agente no llegan IMU ni ToF.')]
    return [Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
                 arguments=['udp4', '--port', LaunchConfiguration('agent_port'), '-v4'],
                 output='screen')]


def twin_bridge_config(source):
    """Copia del bridge con los sensores simulados en /sim/risky/... para no mezclarlos con los reales."""
    with open(source) as f:
        entries = yaml.safe_load(f)
    for entry in entries:
        topic = entry['ros_topic_name']
        if entry['direction'] == 'GZ_TO_ROS' and topic.startswith(REAL_SENSOR_TOPICS):
            entry['ros_topic_name'] = '/sim' + topic
    out = tempfile.NamedTemporaryFile('w', prefix='risky_bridge_', suffix='.yaml', delete=False)
    with out:
        yaml.safe_dump(entries, out, sort_keys=False)
    return out.name


def simulation(context):
    if not is_true(context, 'sim'):
        return []
    # risky_gazebo ya depende de risky_tools; declararlo al revés crearía un ciclo en colcon.
    gazebo_share = get_package_share_directory('risky_gazebo')
    bridge = os.path.join(gazebo_share, 'config', 'bridge.yaml')
    if is_true(context, 'agent') or is_true(context, 'camera'):
        bridge = twin_bridge_config(bridge)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'sim.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz'),
                          'max_thrust_n': LaunchConfiguration('max_thrust_n'),
                          'bridge_config': bridge}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agent', default_value='true'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_url', default_value='http://172.20.10.3:81/stream'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('max_thrust_n', default_value='20.0'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        OpaqueFunction(function=micro_ros_agent),
        Node(package='risky_tools', executable='camera_bridge', output='screen',
             parameters=[{'url': LaunchConfiguration('camera_url')}],
             condition=IfCondition(LaunchConfiguration('camera'))),
        OpaqueFunction(function=simulation),
        Node(package='risky_tools', executable='dashboard', output='screen',
             condition=IfCondition(LaunchConfiguration('dashboard'))),
    ])
