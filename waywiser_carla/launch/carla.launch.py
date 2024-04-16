import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    carla_spawn_objects_file_la = DeclareLaunchArgument(
        'carla_spawn_objects_file',
        default_value=os.path.join(waywiser_carla_dir, 'config/dts_truck_semitrailer.json'),
        description='Full path to carla spawn objects definition file',
    )

    # start nodes and use args to set parameters
    carla_ros_bridge = Node(
        package='carla_ros_bridge',
        executable='bridge',
        name='carla_ros_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('carla_config'),
        ],
    )

    carla_spawn_objects = Node(
        package='carla_spawn_objects',
        executable='carla_spawn_objects',
        name='carla_spawn_objects',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'objects_definition_file': LaunchConfiguration('carla_spawn_objects_file')},
            LaunchConfiguration('carla_config'),
        ],
    )

    carla_initial_pose = Node(
        package='carla_spawn_objects',
        executable='set_initial_pose',
        name='carla_initial_pose',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('carla_config'),
        ],
    )

    carla_twist_to_control = Node(
        package='carla_twist_to_control',
        executable='carla_twist_to_control',
        name='carla_twist_to_control',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('carla_config'),
        ],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(carla_config_la)
    ld.add_action(carla_spawn_objects_file_la)

    # start nodes
    ld.add_action(carla_ros_bridge)
    ld.add_action(carla_spawn_objects)
    ld.add_action(carla_initial_pose)
    ld.add_action(carla_twist_to_control)

    return ld
