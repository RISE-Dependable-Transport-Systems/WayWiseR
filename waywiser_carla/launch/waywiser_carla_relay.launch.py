import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    waywiser_carla_dir = get_package_share_directory('waywiser_carla')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    ego_vehicle_role_name_la = DeclareLaunchArgument(
        'ego_vehicle_role_name',
        default_value='truck',
        description='Name of the ego vehicle',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_carla_dir, 'config/truck_full_scale.yaml'),
        description='Full path to params file of vehicle',
    )

    # start nodes and use args to set parameters
    waywiser_to_carla_control_node = Node(
        package='waywiser_carla',
        executable='waywiser_to_carla_control.py',
        name='waywiser_to_carla_control_node',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
            },
            LaunchConfiguration('vehicle_config'),
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    # create opaque functions to launch nodes using context
    carla_odom_relay_la = OpaqueFunction(function=carla_odom_relay_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(ego_vehicle_role_name_la)
    ld.add_action(vehicle_config_la)

    # start nodes
    ld.add_action(waywiser_to_carla_control_node)
    ld.add_action(carla_odom_relay_la)

    return ld


def carla_odom_relay_launch(context):
    ego_vehicle_role_name = LaunchConfiguration('ego_vehicle_role_name').perform(context)
    input_topic = '/carla/' + ego_vehicle_role_name + '/odometry'

    with open(LaunchConfiguration('vehicle_config').perform(context), 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f)
        node_params = config_data['/**']['ros__parameters']

        if 'odom_topic' in node_params:
            output_topic = node_params['odom_topic']

            relay_node = Node(
                package='topic_tools',
                executable='relay',
                name=ego_vehicle_role_name + '_odom_relay',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                arguments=[input_topic, output_topic],
            )
        else:
            # raise error
            raise ValueError('odom_topic not found in vehicle_config')

    return [relay_node]
