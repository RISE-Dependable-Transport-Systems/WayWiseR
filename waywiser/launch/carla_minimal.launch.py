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
    waywiser_carla_dir = get_package_share_directory('waywiser_carla')
    teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')

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
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='True',
        description='Use Nav2 collision monitoring',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(waywiser_rviz2_dir, 'rviz/map_reference_frame_carla.rviz'),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(teleop_dir, 'config/teleop_sim.yaml'),
        description='Full path to params file',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/truck_full_scale.yaml'),
        description='Full path to params file of vehicle',
    )

    # include launch files
    carla = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_carla_dir,
                    'launch',
                    'carla.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'carla_config': LaunchConfiguration('carla_config'),
            'carla_spawn_objects_file': LaunchConfiguration('carla_spawn_objects_file'),
        }.items(),
    )

    waywiser_carla_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_carla_dir,
                    'launch',
                    'waywiser_carla_relay.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'carla_config': LaunchConfiguration('carla_config'),
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
    ld.add_action(carla_config_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(carla_spawn_objects_file_la)

    # start nodes
    ld.add_action(carla)
    ld.add_action(waywiser_carla_relay)
    ld.add_action(twist_safety)
    ld.add_action(teleop_rviz2)

    return ld
