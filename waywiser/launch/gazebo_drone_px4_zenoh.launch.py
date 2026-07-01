import json
import os
from pathlib import Path
import socket
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from waywiser_description_py.waywiser_description_utils import (
    get_robot_state_publisher_node,
)
from waywiser_gazebo_py import px4_sitl_utils
from waywiser_py.waywiser_utils import shutdown_on_process_error
import yaml


def generate_launch_description():
    waywiser_dir = get_package_share_directory('waywiser')
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    default_gazebo_osm_tile_cache_root = os.path.join(
        os.environ.get('WAYWISER_WS', os.getcwd()),
        'resources',
        'control_tower',
        'gazebo',
    )

    # Zenoh router connection settings (read from environment / .env)
    zenoh_remote_ip = os.environ.get('ZENOH_REMOTE_ROUTER_IP', '127.0.0.1') or '127.0.0.1'
    zenoh_remote_port = os.environ.get('ZENOH_REMOTE_ROUTER_PORT', '7447') or '7447'

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    simulator_nodes_la = DeclareLaunchArgument(
        'simulator_nodes',
        default_value='True',
        description='Launch Gazebo/PX4 and simulator-side drone support nodes',
    )
    gazebo_world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/forest.sdf'),
        description='Full path to gazebo sdf file',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(waywiser_rviz2_dir, 'config/map_reference_frame_drone.rviz'),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop.yaml'),
        description='Full path to params file',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop',
        default_value='True',
        description='Launch teleop',
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
        default_value='Local OSM server',
        description='Control Tower map source: OpenStreetMap, Local OSM server, or None',
    )
    gazebo_osm_tile_server_la = DeclareLaunchArgument(
        'gazebo_osm_tile_server',
        default_value='True',
        description='Serve the Gazebo world as OSM-compatible map tiles',
    )
    gazebo_osm_tile_server_config_la = DeclareLaunchArgument(
        'gazebo_osm_tile_server_config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/gazebo_osm_tile_server.yaml'),
        description='Full path to Gazebo OSM tile server config file',
    )
    gazebo_osm_tile_server_ip_la = DeclareLaunchArgument(
        'gazebo_osm_tile_server_ip',
        default_value='0.0.0.0',
        description='Bind address for the Gazebo OSM tile server',
    )
    gazebo_osm_tile_server_url_la = DeclareLaunchArgument(
        'gazebo_osm_tile_server_url',
        default_value='http://localhost:8081',
        description='Control Tower tile URL for the Gazebo OSM tile server',
    )
    gazebo_osm_tile_cache_dir_la = DeclareLaunchArgument(
        'gazebo_osm_tile_cache_dir',
        default_value=PythonExpression(
            [
                repr(default_gazebo_osm_tile_cache_root + os.sep),
                ' + __import__("os").path.splitext(__import__("os").path.basename("',
                LaunchConfiguration('world'),
                '"))[0]',
            ]
        ),
        description='Control Tower cache directory for Gazebo-served OSM tiles',
    )
    startup_route_file_la = DeclareLaunchArgument(
        'startup_route_file',
        default_value='$WAYWISER_WS/resources/forest.xml',
        description='Route file to load in Control Tower at startup',
    )
    use_nvidia_gpu_la = DeclareLaunchArgument(
        'use_nvidia_gpu',
        default_value='True',
        description='Use NVIDIA PRIME offload environment variables for Gazebo rendering',
    )
    rviz2_la = DeclareLaunchArgument(
        'rviz2',
        default_value='True',
        description='Launch rviz2',
    )
    drone_config_la = DeclareLaunchArgument(
        'drone_config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/drone.yaml'),
        description='Full path to params file of drone',
    )
    drone_vehicle_node_la = DeclareLaunchArgument(
        'drone_vehicle_node',
        default_value='True',
        description='Launch the main WayWiseR drone vehicle node',
    )
    drone_vehicle_node_enable_autopilot_la = DeclareLaunchArgument(
        'drone_vehicle_node_enable_autopilot',
        default_value='False',
        description='Enable the waypoint follower inside the main drone vehicle node',
    )
    drone_control_tower_heartbeat_topic_la = DeclareLaunchArgument(
        'drone_control_tower_heartbeat_topic',
        default_value='',
        description='Override the drone node Control Tower heartbeat input topic when non-empty',
    )
    drone_cmd_vel_out_topic_la = DeclareLaunchArgument(
        'drone_cmd_vel_out_topic',
        default_value='waypoint_follower_vel',
        description='Override the drone node cmd_vel_out publisher topic',
    )
    drone_waypoint_follower_la = DeclareLaunchArgument(
        'drone_waypoint_follower',
        default_value='True',
        description='Launch a command-side copter waypoint follower',
    )
    control_vehicle_node_name_la = DeclareLaunchArgument(
        'control_vehicle_node_name',
        default_value='waywiser_drone_node',
        description='Name of the vehicle node to control',
    )
    drone_spawn_config_file_la = DeclareLaunchArgument(
        'drone_spawn_config_file',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/drone_spawn_config.json'),
        description='Full path to spawn config file',
    )
    drone_yolo_config_la = DeclareLaunchArgument(
        'drone_yolo_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/yolov8.yaml'),
        description='Full path to params file of yolo',
    )
    drone_name_la = DeclareLaunchArgument(
        'drone_name',
        default_value='drone',
        description='Name of the drone, used as ROS namespace and prefix for robot frames',
    )
    drone_localization_node_name_la = DeclareLaunchArgument(
        'drone_localization_node_name',
        default_value='waywiser_drone_localization_node',
        description='Name of the node to be launched',
    )
    px4_sys_autostart_la = DeclareLaunchArgument(
        'px4_sys_autostart',
        default_value='4001',
        description='PX4 SYS_AUTOSTART airframe id (4001 is Gazebo x500).',
    )
    drone_spawn_delay_la = DeclareLaunchArgument(
        'drone_spawn_delay',
        default_value='5.0',
        description='Delay (seconds) before spawning drone models to let Gazebo world initialize.',
    )
    drone_spawn_service_timeout_la = DeclareLaunchArgument(
        'drone_spawn_service_timeout',
        default_value='30000',
        description='Timeout in milliseconds for Gazebo drone spawn service calls.',
    )
    px4_start_delay_la = DeclareLaunchArgument(
        'px4_start_delay',
        default_value='7.0',
        description='Delay (seconds) before starting PX4 after Gazebo starts.',
    )
    launch_gazebo_orchestrator_la = DeclareLaunchArgument(
        'launch_gazebo_orchestrator',
        default_value='True',
        description='Launch the Gazebo setup/reset orchestrator',
    )
    gazebo_orchestrator_config_la = DeclareLaunchArgument(
        'gazebo_orchestrator_config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/gazebo_orchestrator.yaml'),
        description='Full path to Gazebo orchestrator config file',
    )
    drone_name = LaunchConfiguration('drone_name')
    frame_prefix = [drone_name, '/']

    px4_sitl = OpaqueFunction(function=px4_sitl_launch)

    # teleop_rviz2
    teleop_rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_dir,
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
                drone_name,
                '/',
                LaunchConfiguration('control_vehicle_node_name'),
            ],
            'map_source': LaunchConfiguration('map_source'),
            'osm_tile_server_url': LaunchConfiguration('gazebo_osm_tile_server_url'),
            'osm_tile_cache_dir': LaunchConfiguration('gazebo_osm_tile_cache_dir'),
            'startup_route_file': LaunchConfiguration('startup_route_file'),
            'vehicle_control_enabled': LaunchConfiguration('vehicle_control_enabled'),
            'publish_control_tower_heartbeat': LaunchConfiguration(
                'publish_control_tower_heartbeat'
            ),
        }.items(),
    )

    gazebo_osm_tile_server = OpaqueFunction(
        function=gazebo_osm_tile_server_launch,
        condition=IfCondition(LaunchConfiguration('gazebo_osm_tile_server')),
    )

    drone_navsatfix_extended_wrapper = GroupAction(
        actions=[
            PushRosNamespace(drone_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_core_dir,
                            'launch',
                            'navsatfix_extended_wrapper.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'config': LaunchConfiguration('drone_config'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    drone_localization = GroupAction(
        actions=[
            PushRosNamespace(drone_name),
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
                    'localization_config': LaunchConfiguration('drone_config'),
                    'localization_node_name': LaunchConfiguration('drone_localization_node_name'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    drone_twist_safety = GroupAction(
        actions=[
            PushRosNamespace(drone_name),
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
                    'twist_safety_config': LaunchConfiguration('drone_config'),
                }.items(),
            ),
        ]
    )

    drone_yolo = GroupAction(
        actions=[
            PushRosNamespace(drone_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_perception_dir,
                            'launch',
                            'yolo.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'yolo_config': LaunchConfiguration('drone_yolo_config'),
                }.items(),
            ),
        ]
    )

    drone_rgbd_to_pointcloud = GroupAction(
        actions=[
            PushRosNamespace(drone_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_perception_dir,
                            'launch',
                            'rgbd_to_pointcloud.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'rgb_topic': 'sensors/camera/color/image_rect',
                    'depth_topic': 'sensors/camera/aligned_depth_to_color/image_rect',
                    'rgb_camera_info_topic': 'sensors/camera/aligned_depth_to_color/camera_info',
                    'pointcloud_topic': 'sensors/camera/color/points',
                    'max_depth_meters': '10.0',
                    'optical_to_ros_transform': 'True',
                    'node_name': 'rgbd_to_pointcloud',
                }.items(),
            ),
        ]
    )

    drone_state_publisher = OpaqueFunction(function=drone_state_publisher_launch)
    drone_vehicle_node = OpaqueFunction(
        function=drone_vehicle_node_launch,
        condition=IfCondition(LaunchConfiguration('drone_vehicle_node')),
    )
    drone_waypoint_follower = OpaqueFunction(
        function=drone_waypoint_follower_launch,
        condition=IfCondition(LaunchConfiguration('drone_waypoint_follower')),
    )
    simulator_nodes = GroupAction(
        actions=[
            px4_sitl,
            OpaqueFunction(function=normalize_gazebo_osm_tile_cache_dir),
            gazebo_osm_tile_server,
            drone_twist_safety,
            drone_navsatfix_extended_wrapper,
            drone_yolo,
            drone_localization,
            drone_rgbd_to_pointcloud,
            drone_state_publisher,
            drone_vehicle_node,
        ],
        condition=IfCondition(LaunchConfiguration('simulator_nodes')),
    )
    zenoh_router = OpaqueFunction(function=zenoh_router_check)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(simulator_nodes_la)
    ld.add_action(gazebo_world_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(vehicle_control_enabled_la)
    ld.add_action(publish_control_tower_heartbeat_la)
    ld.add_action(map_source_la)
    ld.add_action(gazebo_osm_tile_server_la)
    ld.add_action(gazebo_osm_tile_server_config_la)
    ld.add_action(gazebo_osm_tile_server_ip_la)
    ld.add_action(gazebo_osm_tile_server_url_la)
    ld.add_action(gazebo_osm_tile_cache_dir_la)
    ld.add_action(startup_route_file_la)
    ld.add_action(use_nvidia_gpu_la)
    ld.add_action(rviz2_la)
    ld.add_action(drone_config_la)
    ld.add_action(drone_vehicle_node_la)
    ld.add_action(drone_vehicle_node_enable_autopilot_la)
    ld.add_action(drone_control_tower_heartbeat_topic_la)
    ld.add_action(drone_cmd_vel_out_topic_la)
    ld.add_action(drone_waypoint_follower_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(drone_spawn_config_file_la)
    ld.add_action(drone_yolo_config_la)
    ld.add_action(drone_name_la)
    ld.add_action(drone_localization_node_name_la)
    ld.add_action(px4_sys_autostart_la)
    ld.add_action(drone_spawn_delay_la)
    ld.add_action(drone_spawn_service_timeout_la)
    ld.add_action(px4_start_delay_la)
    ld.add_action(launch_gazebo_orchestrator_la)
    ld.add_action(gazebo_orchestrator_config_la)
    ld.add_action(RegisterEventHandler(OnProcessExit(on_exit=shutdown_on_process_error)))

    # start nodes
    ld.add_action(zenoh_router)
    ld.add_action(SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp'))
    ld.add_action(
        SetEnvironmentVariable(
            'ZENOH_CONFIG_OVERRIDE',
            f'mode="client";connect/endpoints=["tcp/{zenoh_remote_ip}:{zenoh_remote_port}"]',
        )
    )
    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[
                simulator_nodes,
                teleop_rviz2,
                drone_waypoint_follower,
            ],
        )
    )

    return ld


def launch_config_as_bool(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in (
        '1',
        'true',
        'yes',
        'on',
    )


def create_bridge_action(config_file, use_sim_time, bridge_install_prefix=''):
    bridge_binary = px4_sitl_utils.resolve_bridge_executable(bridge_install_prefix)

    if not bridge_binary:
        return Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='log',
            arguments=[
                '--ros-args',
                '-p',
                ['config_file:=', config_file],
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

    bridge_env = {}
    if bridge_install_prefix:
        bridge_env = {
            'LD_LIBRARY_PATH': px4_sitl_utils.prepend_env_path(
                os.environ.get('LD_LIBRARY_PATH', ''),
                str(Path(bridge_install_prefix) / 'lib'),
            ),
            'AMENT_PREFIX_PATH': px4_sitl_utils.prepend_env_path(
                os.environ.get('AMENT_PREFIX_PATH', ''), bridge_install_prefix
            ),
        }

    return ExecuteProcess(
        cmd=[
            str(bridge_binary),
            '--ros-args',
            '-p',
            f'config_file:={config_file}',
            '-p',
            f'use_sim_time:={use_sim_time}',
        ],
        additional_env=bridge_env,
        output='log',
    )


def px4_sitl_launch(context):
    gazebo_dir = get_package_share_directory('waywiser_gazebo')
    world_path = Path(LaunchConfiguration('world').perform(context)).resolve()
    drone_config_path = Path(LaunchConfiguration('drone_config').perform(context)).resolve()
    drone_name = LaunchConfiguration('drone_name').perform(context)
    px4_sys_autostart = LaunchConfiguration('px4_sys_autostart').perform(context)
    px4_start_delay = float(LaunchConfiguration('px4_start_delay').perform(context))
    bridge_install_prefix = px4_sitl_utils.get_vendored_harmonic_bridge_install_prefix()
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_value = LaunchConfiguration('use_sim_time').perform(context)
    gz_world_name = px4_sitl_utils.read_world_name(world_path)
    gazebo_world_path = (
        px4_sitl_utils.create_harmonic_compatible_sdf(world_path)
        if px4_sitl_utils.is_ignition_sdf(world_path)
        else world_path
    )

    px4_sitl_dir = Path(gazebo_dir) / 'px4_sitl_zenoh'
    px4_models_dir = px4_sitl_dir / 'Tools' / 'simulation' / 'gz' / 'models'
    px4_worlds_dir = px4_sitl_dir / 'Tools' / 'simulation' / 'gz' / 'worlds'
    waywiser_description_dir = Path(get_package_share_directory('waywiser_description'))

    enuref = px4_sitl_utils.read_enuref(drone_config_path)
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_entries = [entry for entry in existing_gz_resource_path.split(':') if entry]
    gz_resource_entries.extend(
        [
            str(px4_models_dir),
            str(px4_worlds_dir),
            str(world_path.parent),
            str(gazebo_world_path.parent),
            str(waywiser_description_dir / 'sdf'),
            str(waywiser_description_dir.parent),
        ]
    )

    deduped_resource_entries = []
    for entry in gz_resource_entries:
        if entry not in deduped_resource_entries:
            deduped_resource_entries.append(entry)

    px4_build_dir = px4_sitl_utils.prepare_writable_px4_runtime_dir(px4_sitl_dir)
    px4_rootfs_dir = px4_build_dir / 'rootfs'
    px4_binary = px4_build_dir / 'bin' / 'px4'

    if not px4_binary.is_file():
        raise RuntimeError(
            f"PX4 binary not found at '{px4_binary}'. Build waywiser_gazebo first so "
            'PX4 SITL Zenoh artifacts are generated.'
        )

    px4_sitl_utils.validate_gazebo_compatibility(px4_binary, px4_sitl_dir, bridge_install_prefix)

    px4_param_overrides = {
        'NAV_DLL_ACT': '0',
        'COM_DLL_EXCEPT': '4',
        'COM_RCL_EXCEPT': '4',
        'COM_ARM_WO_GPS': '1',
        'COM_ARM_CHK_ESCS': '0',
        'FD_ESCS_EN': '0',
        'SYS_FAILURE_EN': '0',
        'CBRK_FLIGHTTERM': '121212',
        'CBRK_SUPPLY_CHK': '894281',
        'COM_DISARM_PRFLT': '-1',
        'COM_DISARM_LAND': '2',
        'COM_LOW_BAT_ACT': '0',
        'SIM_GZ_EN': '1',
        'SIM_GZ_EC_FUNC1': '101',
        'SIM_GZ_EC_FUNC2': '102',
        'SIM_GZ_EC_FUNC3': '103',
        'SIM_GZ_EC_FUNC4': '104',
        'SIM_GZ_EC_MIN1': '150',
        'SIM_GZ_EC_MIN2': '150',
        'SIM_GZ_EC_MIN3': '150',
        'SIM_GZ_EC_MIN4': '150',
        'SIM_GZ_EC_MAX1': '1000',
        'SIM_GZ_EC_MAX2': '1000',
        'SIM_GZ_EC_MAX3': '1000',
        'SIM_GZ_EC_MAX4': '1000',
    }
    drone_config_data = yaml_to_dict(drone_config_path)
    drone_node_params = drone_config_data.get('waywiser_drone_node', {}).get('ros__parameters', {})
    rtl_horizontal_velocity = drone_node_params.get('mission_cruise_speed')
    if rtl_horizontal_velocity is not None:
        px4_param_overrides['MPC_XY_CRUISE'] = str(max(0.0, float(rtl_horizontal_velocity)))
    rtl_max_horizontal_velocity = drone_node_params.get('mission_max_speed')
    if rtl_max_horizontal_velocity is not None:
        px4_param_overrides['MPC_XY_VEL_MAX'] = str(max(0.0, float(rtl_max_horizontal_velocity)))
    px4_sitl_utils.refresh_px4_zenoh_runtime_config(
        px4_build_dir,
        px4_rootfs_dir,
        px4_sys_autostart,
        px4_param_overrides,
    )

    px4_env = {
        'PX4_GZ_WORLDS': str(world_path.parent),
        'PX4_GZ_WORLD': gz_world_name,
        'PX4_GZ_MODELS': str(px4_models_dir),
        'GZ_SIM_RESOURCE_PATH': ':'.join(deduped_resource_entries),
        'PX4_HOME_LAT': str(enuref[0]),
        'PX4_HOME_LON': str(enuref[1]),
        'PX4_HOME_ALT': str(enuref[2]),
        'PX4_SYS_AUTOSTART': str(px4_sys_autostart),
        'PX4_GZ_STANDALONE': '1',
        'PX4_PARAM_ZENOH_ENABLE': '1',
        'PX4_PARAM_ZENOH_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
        'PX4_GZ_MODEL_NAME': drone_name,
    }
    px4_env.update({f'PX4_PARAM_{name}': value for name, value in px4_param_overrides.items()})

    default_bridge_config = px4_sitl_utils.create_runtime_bridge_config(
        os.path.join(gazebo_dir, 'config', 'default_gazebo_bridges.yaml'), gz_world_name
    )
    model_bridge_config = px4_sitl_utils.create_runtime_bridge_config(
        os.path.join(gazebo_dir, 'config', 'drone_gazebo_bridges.yaml'), gz_world_name
    )
    spawn_config_file = prepare_drone_spawn_config(context)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_dir, 'launch', 'gazebo_orchestrator.launch.py')]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': str(gazebo_world_path),
            'launch_bridge': 'False',
            'launch_map_frame_transform': 'False',
            'gazebo_sim_version': '8',
            'use_nvidia_gpu': LaunchConfiguration('use_nvidia_gpu'),
            'launch_gazebo_orchestrator': LaunchConfiguration('launch_gazebo_orchestrator'),
            'gazebo_orchestrator_config': LaunchConfiguration('gazebo_orchestrator_config'),
            'manage_px4_process': 'True',
            'px4_command_json': json.dumps([str(px4_binary), str(px4_build_dir / 'etc')]),
            'px4_working_directory': str(px4_rootfs_dir),
            'px4_environment_json': json.dumps(px4_env),
            'px4_start_delay_sec': str(px4_start_delay),
            'service_timeout_ms': LaunchConfiguration('drone_spawn_service_timeout'),
            'spawn_config_file': spawn_config_file,
            'spawn_on_startup': 'True',
            'spawn_start_delay_sec': LaunchConfiguration('drone_spawn_delay'),
            'spawn_backend': 'gz_service',
            'start_gazebo_bridge': 'False',
            'gz_service_suppress_output': 'True',
        }.items(),
    )

    map_frame_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--roll',
            '0',
            '--pitch',
            '0',
            '--yaw',
            '0',
            '--frame-id',
            gz_world_name,
            '--child-frame-id',
            'map',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    px4_sitl_process = ExecuteProcess(
        cmd=[str(px4_binary), str(px4_build_dir / 'etc')],
        cwd=str(px4_rootfs_dir),
        additional_env=px4_env,
        output='screen',
        emulate_tty=True,
    )

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(deduped_resource_entries)),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join(deduped_resource_entries)),
        gazebo,
        map_frame_transform,
        create_bridge_action(default_bridge_config, use_sim_time_value, bridge_install_prefix),
        create_bridge_action(model_bridge_config, use_sim_time_value, bridge_install_prefix),
    ]
    if not launch_config_as_bool(context, 'launch_gazebo_orchestrator'):
        actions.append(TimerAction(period=px4_start_delay, actions=[px4_sitl_process]))
    return actions


def normalize_gazebo_osm_tile_cache_dir(context):
    cache_dir = LaunchConfiguration('gazebo_osm_tile_cache_dir').perform(context)
    cache_name = os.path.basename(os.path.normpath(cache_dir))
    if not cache_name.startswith('waywiser_harmonic_'):
        return []

    world_path = (
        Path(os.path.expandvars(LaunchConfiguration('world').perform(context)))
        .expanduser()
        .resolve()
    )
    world_name = read_world_name(world_path)
    stable_cache_dir = os.path.join(os.path.dirname(os.path.normpath(cache_dir)), world_name)
    return [
        LogInfo(
            msg=(
                f'Ignoring runtime-generated Gazebo OSM tile cache directory {cache_dir}; '
                f'using {stable_cache_dir}.'
            )
        ),
        SetLaunchConfiguration('gazebo_osm_tile_cache_dir', stable_cache_dir),
    ]


def drone_state_publisher_launch(context):
    drone_config = LaunchConfiguration('drone_config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in [
        'true',
        '1',
        'yes',
    ]

    drone_name = LaunchConfiguration('drone_name').perform(context)
    frame_prefix = drone_name + '/'

    # Robot State Publisher for drone
    config_data = yaml_to_dict(drone_config)
    rsp_params = config_data['/**']['ros__parameters']
    rsp_node = get_robot_state_publisher_node(context, rsp_params, use_sim_time, frame_prefix)

    return [
        GroupAction(
            actions=[
                PushRosNamespace(drone_name),
                SetRemap(src='/tf', dst='/tf'),
                SetRemap(src='/tf_static', dst='/tf_static'),
                rsp_node,
            ]
        )
    ]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r', encoding='utf-8') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def read_world_name(world_path: Path):
    if world_path.is_file():
        try:
            tree = ET.parse(world_path)
            root = tree.getroot()
            world_element = root if root.tag == 'world' else root.find('world')
            if world_element is not None and world_element.get('name'):
                return str(world_element.get('name'))
        except (ET.ParseError, OSError):
            pass
    return world_path.stem


def resolve_resource_path(path, base_dir):
    path = str(path)
    if path.startswith('package://'):
        package_path = path[len('package://'):]
        package_name, _, relative_path = package_path.partition('/')
        if not package_name or not relative_path:
            raise RuntimeError(f'Invalid package resource URI: {path}')
        return str(Path(get_package_share_directory(package_name)) / relative_path)

    candidate = Path(path)
    if candidate.is_absolute() or candidate.exists():
        return str(candidate)
    return str(base_dir / candidate)


def prepare_drone_spawn_config(context):
    """Create the PX4-compatible spawn config consumed by the orchestrator."""
    config_path = LaunchConfiguration('drone_spawn_config_file').perform(context)
    drone_name = LaunchConfiguration('drone_name').perform(context)
    config_dir = Path(config_path).resolve().parent
    with open(config_path) as f:
        config = json.load(f)

    # Apply PX4-specific SDF transforms to each model before spawning.
    for model in config.get('sdf_models', []):
        model.setdefault('name', drone_name)
        path = model.get('path')
        if path:
            sdf_path = resolve_resource_path(path, config_dir)
            sdf_path = px4_sitl_utils.create_sdf_without_multicopter_velocity_control(sdf_path)
            sdf_path = px4_sitl_utils.create_sdf_with_px4_sim_sensors(sdf_path)
            sdf_path = px4_sitl_utils.create_sdf_with_px4_motor_joint_names(sdf_path)
            sdf_path = px4_sitl_utils.create_model_sdf_with_harmonic_plugins(sdf_path)
            model['path'] = sdf_path

    temp_config = tempfile.NamedTemporaryFile(
        mode='w', prefix='waywiser_px4_spawn_', suffix='.json', delete=False
    )
    with temp_config:
        json.dump(config, temp_config)
    return temp_config.name


def gazebo_osm_tile_server_launch(context):
    config_path = LaunchConfiguration('gazebo_osm_tile_server_config').perform(context)
    config_data = yaml_to_dict(config_path)
    node_params = config_data.get('gazebo_osm_tile_server_node', {}).get('ros__parameters', {})
    node_params = {
        **node_params,
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower()
        in ['true', '1', 'yes'],
        'tcp_server_ip': LaunchConfiguration('gazebo_osm_tile_server_ip').perform(context),
        'world_sdf': LaunchConfiguration('world').perform(context),
        'base_map_cache_dir': LaunchConfiguration('gazebo_osm_tile_cache_dir').perform(context),
    }

    return [
        Node(
            package='waywiser_gazebo',
            executable='gazebo_osm_tile_server_node.py',
            name='gazebo_osm_tile_server_node',
            parameters=[node_params],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen',
            emulate_tty=True,
        )
    ]


def zenoh_router_check(context):
    # Auto-start is intentionally disabled. Require a user-managed router.
    port = int(os.environ.get('ZENOH_REMOTE_ROUTER_PORT', '7447') or '7447')
    is_router_running = False
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.1)
            if s.connect_ex(('127.0.0.1', port)) == 0:
                is_router_running = True
    except Exception:
        pass

    if is_router_running:
        return [LogInfo(msg=f'Zenoh router detected on port {port}. Proceeding with launch.')]

    shutdown_reason = (
        'Zenoh router is not running. Start it in a separate terminal, then relaunch.'
    )

    return [
        LogInfo(msg=shutdown_reason),
        EmitEvent(event=Shutdown(reason=shutdown_reason)),
    ]


def drone_vehicle_node_launch(context):
    enable_autopilot = LaunchConfiguration('drone_vehicle_node_enable_autopilot').perform(
        context
    ).lower() in ['true', '1', 'yes']
    parameter_overrides = {
        'enable_autopilot_component': enable_autopilot,
        'enable_px4_bridge': True,
    }
    control_tower_heartbeat_topic = LaunchConfiguration(
        'drone_control_tower_heartbeat_topic'
    ).perform(context)
    if control_tower_heartbeat_topic:
        parameter_overrides['control_tower_heartbeat_topic'] = control_tower_heartbeat_topic

    cmd_vel_out_topic = LaunchConfiguration('drone_cmd_vel_out_topic').perform(context)

    remappings = [
        ('/cmd_vel_in', 'twist_safety_vel'),
        ('/cmd_vel_out', cmd_vel_out_topic),
    ]
    use_waypoint_follower = LaunchConfiguration('drone_waypoint_follower').perform(
        context
    ).lower() in ['true', '1', 'yes']
    if not enable_autopilot and not use_waypoint_follower:
        # No autopilot component and no separate waypoint follower: nobody publishes
        # mission_status, so redirect the subscriber to a dummy topic.
        remappings.append(('mission_status', 'mission_status_dummy'))

    return create_drone_vehicle_node(
        context,
        node_name=LaunchConfiguration('control_vehicle_node_name'),
        parameter_overrides=parameter_overrides,
        remappings=remappings,
    )


def drone_waypoint_follower_launch(context):
    control_tower_heartbeat_topic = LaunchConfiguration(
        'drone_control_tower_heartbeat_topic'
    ).perform(context)
    parameter_overrides = {
        'enable_autopilot_component': True,
        'enable_px4_bridge': False,
        'auto_arm': False,
        'auto_lift_off': False,
        'input_odom_topic': 'odometry',
        'odom_topic': '',
        'vehicle_pose_topic': '',
        'quadcopter_state_topic': '',
        'battery_state_topic': '',
        'arm_command_topic': '',
        'emergency_stop_update_topic': '',
        'control_tower_heartbeat_rx_state_topic': '',
        'joint_states_topic': '',
        'min_steering_height': 0.0,
        'publish_odom_to_baselink_tf': False,
        'publish_world_to_odom_tf': False,
    }
    if control_tower_heartbeat_topic:
        parameter_overrides['control_tower_heartbeat_topic'] = control_tower_heartbeat_topic

    cmd_vel_out_topic = LaunchConfiguration('drone_cmd_vel_out_topic').perform(context)
    return create_drone_vehicle_node(
        context,
        node_name='waywiser_drone_waypoint_follower',
        parameter_overrides=parameter_overrides,
        remappings=[
            ('/cmd_vel_in', 'waypoint_follower_cmd_vel_in'),
            ('/cmd_vel_out', cmd_vel_out_topic),
        ],
    )


def create_drone_vehicle_node(context, node_name, parameter_overrides, remappings):
    drone_config = LaunchConfiguration('drone_config').perform(context)
    drone_name = LaunchConfiguration('drone_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in [
        'true',
        '1',
        'yes',
    ]

    config_data = yaml_to_dict(drone_config)
    shared_params = config_data.get('/**', {}).get('ros__parameters', {})
    node_specific_params = config_data.get('waywiser_drone_node', {}).get('ros__parameters', {})
    node_params = {**shared_params, **node_specific_params, **parameter_overrides}

    # Get the processed URDF string for the vehicle node as well
    from waywiser_description_py.waywiser_description_utils import (
        get_scaled_urdf_string,
    )

    urdf_scale = node_params.get('urdf_scale', 1.0)
    urdf_extra_args = node_params.get('urdf_extra_args', '')
    urdf_string = get_scaled_urdf_string(
        context, node_params['urdf_file'], urdf_scale, urdf_extra_args, f'{drone_name}/'
    )

    return [
        GroupAction(
            actions=[
                PushRosNamespace(drone_name),
                SetRemap(src='/tf', dst='/tf'),
                SetRemap(src='/tf_static', dst='/tf_static'),
                Node(
                    package='waywiser_core',
                    executable='waywiser_copter_node',
                    name=node_name,
                    parameters=[
                        node_params,
                        {
                            'use_sim_time': use_sim_time,
                            'frame_prefix': f'{drone_name}/',
                            'urdf_file': urdf_string,
                        },
                    ],
                    remappings=remappings,
                    output='screen',
                ),
            ]
        )
    ]
