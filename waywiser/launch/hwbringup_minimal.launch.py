import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )
    rover_config_la = DeclareLaunchArgument(
        'rover_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/rover.yaml'),
        description='Full path to params file of rover',
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='True',
        description='Use Nav2 collision monitoring',
    )

    # include launch files
    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_hwbringup_dir,
                    'launch',
                    'rover.launch.py',
                )
            ]
        ),
        launch_arguments={
            'rover_config': LaunchConfiguration('rover_config'),
        }.items(),
    )

    twist_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_twist_safety_dir,
                    'launch',
                    'twist_safety.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_collision_monitor': LaunchConfiguration('enable_collision_monitor'),
        }.items(),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(rover_config_la)
    ld.add_action(enable_collision_monitor_la)

    # start nodes
    ld.add_action(rover)
    ld.add_action(twist_safety)

    return ld
