import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    waywiser_carla_dir = get_package_share_directory('waywiser_carla')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    carla_config_la = DeclareLaunchArgument(
        'carla_config',
        default_value=os.path.join(waywiser_carla_dir, 'config/carla.yaml'),
        description='Full path to params file for carla',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_carla_dir, 'config/truck_full_scale.yaml'),
        description='Full path to params file of truck',
    )

    # start nodes and use args to set parameters
    waywiser_twist_to_carla_control_node = Node(
        package='waywiser_carla',
        executable='waywiser_twist_to_carla_control.py',
        name='waywiser_twist_to_carla_control_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('carla_config'),
            LaunchConfiguration('vehicle_config'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    carla_map_to_odom_tf_publisher_la = OpaqueFunction(
        function=carla_map_to_odom_tf_publisher_launch
    )
    emulated_angle_sensor_conditional_launch_action = OpaqueFunction(
        function=emulated_angle_sensor_conditional_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(carla_config_la)
    ld.add_action(vehicle_config_la)

    # start nodes
    ld.add_action(waywiser_twist_to_carla_control_node)
    ld.add_action(carla_map_to_odom_tf_publisher_la)
    ld.add_action(emulated_angle_sensor_conditional_launch_action)

    return ld


def carla_map_to_odom_tf_publisher_launch(context):
    ego_vehicle_role_name = ''
    with open(LaunchConfiguration('carla_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        carla_params_dict = config_data['/**']['ros__parameters']
        if 'ego_vehicle_role_name' in carla_params_dict:
            ego_vehicle_role_name = carla_params_dict['ego_vehicle_role_name']

    carla_map_to_odom_tf_publisher = Node(
        package='waywiser_carla',
        executable='map_to_odom_tf_publisher',
        name='carla_map_to_odom_tf_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'base_link_frame': ego_vehicle_role_name},
            {'odom_frame': 'odom'},
            {'odom_topic': '/carla/' + ego_vehicle_role_name + '/odometry'},
        ],
    )

    return [carla_map_to_odom_tf_publisher]


def emulated_angle_sensor_conditional_launch(context):
    enable_emulated_angle_sensor = False
    with open(LaunchConfiguration('vehicle_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        emulated_angle_sensor_params_dict = config_data['emulated_angle_sensor']['ros__parameters']
        if 'enable' in emulated_angle_sensor_params_dict:
            enable_emulated_angle_sensor = emulated_angle_sensor_params_dict['enable']

    emulated_angle_sensor_node = Node(
        package='waywiser_carla',
        executable='emulated_angle_sensor',
        name='emulated_angle_sensor',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('vehicle_config'),
        ],
        output='screen',
        condition=IfCondition(str(enable_emulated_angle_sensor)),
    )

    return [emulated_angle_sensor_node]
