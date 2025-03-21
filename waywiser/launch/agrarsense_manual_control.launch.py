import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_agrarsense_dir = get_package_share_directory('waywiser_agrarsense')
    teleop_dir = get_package_share_directory('waywiser_teleop')

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
    agrarsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_agrarsense_dir,
                    'launch',
                    'agrarsense.launch.py',
                )
            ]
        ),
        launch_arguments={
            'sim_config': LaunchConfiguration('sim_config'),
        }.items(),
    )

    waywiser_agrarsense_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_agrarsense_dir,
                    'launch',
                    'waywiser_agrarsense_relay.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sim_config': LaunchConfiguration('sim_config'),
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'ego_vehicle_identifier': LaunchConfiguration('ego_vehicle_identifier'),
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
            'twist_safety_config': LaunchConfiguration('vehicle_config'),
        }.items(),
    )

    # include launch files
    teleop_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('waywiser'),
                    'launch',
                    'teleop_rviz2.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
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
    ld.add_action(agrarsense)
    ld.add_action(twist_safety)
    ld.add_action(teleop_rviz2)
    ld.add_action(waywiser_agrarsense_relay)

    return ld
