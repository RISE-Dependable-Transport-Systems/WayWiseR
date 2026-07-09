import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_carla_dir = get_package_share_directory('waywiser_carla')
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock'
    )
    ego_vehicle_role_name_la = DeclareLaunchArgument(
        'ego_vehicle_role_name',
        default_value='truck',
        description='Name of the ego vehicle',
    )
    carla_orchestrator_config_la = DeclareLaunchArgument(
        'carla_orchestrator_config',
        default_value=os.path.join(waywiser_carla_dir, 'config/carla_orchestrator.yaml'),
        description='Full path to params file for carla orchestrator',
    )
    carla_spawn_objects_file_la = DeclareLaunchArgument(
        'carla_spawn_objects_file',
        default_value=os.path.join(waywiser_carla_dir, 'config/dts_truck_semitrailer.json'),
        description='Full path to carla spawn objects definition file',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(waywiser_rviz2_dir, 'config/map_reference_frame_carla.rviz'),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_carla_dir, 'config/truck_full_scale.yaml'),
        description='Full path to params file of vehicle',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop',
        default_value='True',
        description='Launch teleop',
    )
    rviz2_la = DeclareLaunchArgument(
        'rviz2',
        default_value='True',
        description='Launch rviz2',
    )
    control_vehicle_node_name_la = DeclareLaunchArgument(
        'control_vehicle_node_name',
        default_value='waywiser_truck_node',
        description='Name of the vehicle node to control',
    )
    localization_node_name_la = DeclareLaunchArgument(
        'localization_node_name',
        default_value='waywiser_truck_localization_node',
        description='Name of the node to be launched',
    )
    vehicle_name_la = DeclareLaunchArgument(
        'vehicle_name',
        default_value='semitruck',
        description='Name of the vehicle',
    )
    vehicle_control_enabled_la = DeclareLaunchArgument(
        'vehicle_control_enabled',
        default_value='True',
        description='Enable Control Tower vehicle controls; set False for passive monitoring',
    )
    publish_control_tower_heartbeat_la = DeclareLaunchArgument(
        'publish_control_tower_heartbeat',
        default_value='True',
        description='Publish the vehicle-scoped Control Tower heartbeat',
    )
    map_source_la = DeclareLaunchArgument(
        'map_source',
        default_value='OpenStreetMap',
        description='Control Tower map source: OpenStreetMap, Local OSM server, or None',
    )
    osm_tile_server_url_la = DeclareLaunchArgument(
        'osm_tile_server_url',
        default_value='',
        description='Control Tower OSM tile server URL',
    )
    osm_tile_cache_dir_la = DeclareLaunchArgument(
        'osm_tile_cache_dir',
        default_value='',
        description='Control Tower OSM tile cache directory',
    )
    startup_route_file_la = DeclareLaunchArgument(
        'startup_route_file',
        default_value='',
        description='Route file to load in Control Tower at startup',
    )

    vehicle_name = LaunchConfiguration('vehicle_name')
    frame_prefix = [vehicle_name, '/']

    # include launch files
    waywiser_truck_launch = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_core_dir,
                            'launch',
                            'waywiser_truck.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'vehicle_config': LaunchConfiguration('vehicle_config'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    vehicle_tf_navsatfix_extended_wrapper = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_core'),
                            'launch',
                            'navsatfix_extended_wrapper.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'config': LaunchConfiguration('vehicle_config'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    vehicle_localization = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_core_dir,
                            'launch',
                            'waywiser_localization.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'localization_config': LaunchConfiguration('vehicle_config'),
                    'localization_node_name': LaunchConfiguration('localization_node_name'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    vehicle_twist_safety = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
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
                    'enable_nav2_collision_monitor': 'False',
                    'twist_safety_config': LaunchConfiguration('vehicle_config'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    carla_orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_carla_dir,
                    'launch',
                    'carla_orchestrator.launch.py',
                )
            ]
        ),
        launch_arguments={
            'config': LaunchConfiguration('carla_orchestrator_config'),
            'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
        }.items(),
    )

    waywiser_carla_relay = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
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
                    'vehicle_config': LaunchConfiguration('vehicle_config'),
                    'ego_vehicle_role_name': LaunchConfiguration('ego_vehicle_role_name'),
                }.items(),
            ),
        ]
    )

    emulated_angle_sensor = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_carla_dir,
                            'launch',
                            'emulated_angle_sensor.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'config': LaunchConfiguration('vehicle_config'),
                }.items(),
            ),
        ]
    )

    emulated_range_sensor_array = GroupAction(
        actions=[
            PushRosNamespace(vehicle_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_carla_dir,
                            'launch',
                            'emulated_range_sensor_array.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'config': LaunchConfiguration('vehicle_config'),
                }.items(),
            ),
        ]
    )

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
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
            'control_vehicle_node_fqn': [
                vehicle_name,
                '/',
                LaunchConfiguration('control_vehicle_node_name'),
            ],
            'vehicle_control_enabled': LaunchConfiguration('vehicle_control_enabled'),
            'publish_control_tower_heartbeat': LaunchConfiguration(
                'publish_control_tower_heartbeat'
            ),
            'map_source': LaunchConfiguration('map_source'),
            'osm_tile_server_url': LaunchConfiguration('osm_tile_server_url'),
            'osm_tile_cache_dir': LaunchConfiguration('osm_tile_cache_dir'),
            'startup_route_file': LaunchConfiguration('startup_route_file'),
        }.items(),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(ego_vehicle_role_name_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(carla_orchestrator_config_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(carla_spawn_objects_file_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(localization_node_name_la)
    ld.add_action(vehicle_name_la)
    ld.add_action(vehicle_control_enabled_la)
    ld.add_action(publish_control_tower_heartbeat_la)
    ld.add_action(map_source_la)
    ld.add_action(osm_tile_server_url_la)
    ld.add_action(osm_tile_cache_dir_la)
    ld.add_action(startup_route_file_la)

    # start nodes
    ld.add_action(carla_orchestrator)
    ld.add_action(waywiser_carla_relay)
    ld.add_action(vehicle_twist_safety)
    ld.add_action(waywiser_truck_launch)
    ld.add_action(vehicle_localization)
    ld.add_action(emulated_angle_sensor)
    ld.add_action(emulated_range_sensor_array)
    ld.add_action(teleop_rviz2)
    ld.add_action(vehicle_tf_navsatfix_extended_wrapper)

    return ld
