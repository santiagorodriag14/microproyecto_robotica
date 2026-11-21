"""Robot real y simulación juntos: agente micro-ROS (IMU y ToF), puente de la ESP32-CAM,
Gazebo con RViz y panel de datos.

Con agente y simulación a la vez se arma el gemelo digital: twin_estimator calcula la pose del
robot real y mueve con ella el modelo de Gazebo y el de RViz.

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
from launch_ros.parameter_descriptions import ParameterValue
import yaml

REAL_SENSOR_TOPICS = ('/risky/imu/', '/risky/tof/', '/risky/camera/')
# En el gemelo los publica twin_estimator, no la física de Gazebo.
TWIN_OWNED_TOPICS = ('/risky/odom', '/tf', '/joint_states')


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


def twin_bridge_config(source, twin):
    """Copia del bridge con los sensores simulados en /sim/risky/... para no mezclarlos con los reales."""
    with open(source) as f:
        entries = yaml.safe_load(f)
    if twin:
        entries = [e for e in entries
                   if e['direction'] == 'GZ_TO_ROS' and e['ros_topic_name'] not in TWIN_OWNED_TOPICS]
        entries.append({'ros_topic_name': '/sim/twin/front_joint',
                        'gz_topic_name': '/model/risky/joint/Chasis_frontal_Joint/cmd_pos',
                        'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double',
                        'direction': 'ROS_TO_GZ'})
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
    twin = is_true(context, 'agent')
    if twin or is_true(context, 'camera'):
        bridge = twin_bridge_config(bridge, twin)
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'sim.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz'),
                          'max_thrust_n': LaunchConfiguration('max_thrust_n'),
                          'twin': str(twin).lower(),
                          'bridge_config': bridge}.items())]
    if twin:
        actions.append(Node(package='risky_tools', executable='twin_estimator', output='screen',
                            parameters=[{'speed_scale': ParameterValue(LaunchConfiguration('speed_scale'),
                                                                       value_type=float),
                                         'yaw0': ParameterValue(LaunchConfiguration('yaw0'),
                                                                value_type=float)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agent', default_value='true'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('camera_url', default_value='http://172.20.10.3:81/stream'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('max_thrust_n', default_value='20.0'),
        # Factor sobre la velocidad de /risky/cmd_vel: distancia real recorrida / distancia del gemelo.
        DeclareLaunchArgument('speed_scale', default_value='1.0'),
        # Rumbo con el que se enciende el robot: 0 mirando a la pared, 3.1416 de espaldas a ella.
        DeclareLaunchArgument('yaw0', default_value='0.0'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        OpaqueFunction(function=micro_ros_agent),
        Node(package='risky_tools', executable='camera_bridge', output='screen',
             parameters=[{'url': LaunchConfiguration('camera_url')}],
             condition=IfCondition(LaunchConfiguration('camera'))),
        OpaqueFunction(function=simulation),
        Node(package='risky_tools', executable='dashboard', output='screen',
             condition=IfCondition(LaunchConfiguration('dashboard'))),
    ])
