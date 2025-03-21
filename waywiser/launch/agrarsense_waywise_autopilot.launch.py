import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_agrarsense_dir = get_package_share_directory('waywiser_agrarsense')
    waywiser_dir = get_package_share_directory('waywiser')
    teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='True',
        description='Use Nav2 collision monitoring',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            waywiser_rviz2_dir, 'config/map_reference_frame_agrarsense.rviz'
        ),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(teleop_dir, 'config/teleop_sim.yaml'),
        description='Full path to params file',
    )
    sim_config_la = DeclareLaunchArgument(
        'sim_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/agrarsense.yaml'),
        description='Full path to params file of orchestrator',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_agrarsense_dir, 'config/forwarder.yaml'),
        description='Full path to params file of vehicle',
    )
    ego_vehicle_identifier_la = DeclareLaunchArgument(
        'ego_vehicle_identifier',
        default_value='forwarder',
        description='Identifier of ego vehicle',
    )

    # include launch files
    agrarsense_manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_dir,
                    'launch',
                    'agrarsense_manual_control.launch.py',
                )
            ]
        ),
        launch_arguments={
            'sim_config': LaunchConfiguration('sim_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'ego_vehicle_identifier': LaunchConfiguration('ego_vehicle_identifier'),
            'enable_collision_monitor': LaunchConfiguration('enable_collision_monitor'),
        }.items(),
    )

    waywise_autopilot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_hwbringup_dir,
                    'launch',
                    'waywise_car_autopilot.launch.py',
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
    ld.add_action(sim_config_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(ego_vehicle_identifier_la)

    # start nodes
    ld.add_action(agrarsense_manual_control)
    ld.add_action(waywise_autopilot)

    return ld
