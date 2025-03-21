import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_dir = get_package_share_directory('waywiser')
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    teleop_dir = get_package_share_directory('waywiser_teleop')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation clock'
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/truck_small_scale.yaml'),
        description='Full path to params file of vehicle',
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='False',
        description='Use Nav2 collision monitoring',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop',
        default_value='False',
        description='Launch teleop',
    )
    rviz2_la = DeclareLaunchArgument(
        'rviz2',
        default_value='False',
        description='Launch rviz2',
    )

    # include launch files
    truck_manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_dir,
                    'launch',
                    'truck_manual_control.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'enable_collision_monitor': LaunchConfiguration('enable_collision_monitor'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
        }.items(),
    )

    waywise_autopilot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_hwbringup_dir,
                    'launch',
                    'waywise_truck_autopilot.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
        }.items(),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(enable_collision_monitor_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)

    # start nodes
    ld.add_action(truck_manual_control)
    ld.add_action(waywise_autopilot)

    return ld
