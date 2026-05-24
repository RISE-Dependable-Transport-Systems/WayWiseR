import importlib.util
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
from launch_ros.parameter_descriptions import ParameterValue

from waywiser_description_py.waywiser_description_utils import (
    get_robot_state_publisher_node,
)
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
        default_value='$WAYWISER_WS/resources/zigzag.xml',
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
        default_value='True',
        description='Enable the waypoint follower inside the main drone vehicle node',
    )
    drone_waypoint_follower_la = DeclareLaunchArgument(
        'drone_waypoint_follower',
        default_value='False',
        description='Launch a command-side copter waypoint follower that publishes velocity commands',
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
    drone_name = LaunchConfiguration('drone_name')
    frame_prefix = [drone_name, '/']

    px4_sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_gazebo_dir,
                    'launch',
                    'px4_sitl.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'default_gazebo_bridge': os.path.join(
                waywiser_gazebo_dir, 'config', 'default_gazebo_bridges.yaml'
            ),
            'gazebo_bridge': os.path.join(
                waywiser_gazebo_dir, 'config', 'drone_gazebo_bridges.yaml'
            ),
            'drone_config': LaunchConfiguration('drone_config'),
            'drone_name': LaunchConfiguration('drone_name'),
            'px4_sys_autostart': LaunchConfiguration('px4_sys_autostart'),
            'px4_start_delay': LaunchConfiguration('px4_start_delay'),
            'use_nvidia_gpu': LaunchConfiguration('use_nvidia_gpu'),
        }.items(),
    )

    drone_gazebo_spawn = OpaqueFunction(function=drone_gazebo_spawn_launch)

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
        }.items(),
    )

    gazebo_osm_tile_server = Node(
        package='waywiser_gazebo',
        executable='gazebo_osm_tile_server.py',
        name='gazebo_osm_tile_server_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('gazebo_osm_tile_server_config'),
            {'world_sdf': ParameterValue(LaunchConfiguration('world'), value_type=str)},
            {
                'base_map_cache_dir': ParameterValue(
                    LaunchConfiguration('gazebo_osm_tile_cache_dir'), value_type=str
                )
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
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
            drone_gazebo_spawn,
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
    ld.add_action(map_source_la)
    ld.add_action(gazebo_osm_tile_server_la)
    ld.add_action(gazebo_osm_tile_server_config_la)
    ld.add_action(gazebo_osm_tile_server_url_la)
    ld.add_action(gazebo_osm_tile_cache_dir_la)
    ld.add_action(startup_route_file_la)
    ld.add_action(use_nvidia_gpu_la)
    ld.add_action(rviz2_la)
    ld.add_action(drone_config_la)
    ld.add_action(drone_vehicle_node_la)
    ld.add_action(drone_vehicle_node_enable_autopilot_la)
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
        package_path = path[len('package://') :]
        package_name, _, relative_path = package_path.partition('/')
        if not package_name or not relative_path:
            raise RuntimeError(f'Invalid package resource URI: {path}')
        return str(Path(get_package_share_directory(package_name)) / relative_path)

    candidate = Path(path)
    if candidate.is_absolute() or candidate.exists():
        return str(candidate)
    return str(base_dir / candidate)


def drone_gazebo_spawn_launch(context):
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')

    # Load PX4 SDF transform utilities from px4_sitl.launch.py.
    px4_sitl_path = os.path.join(waywiser_gazebo_dir, 'launch', 'px4_sitl.launch.py')
    spec = importlib.util.spec_from_file_location('px4_sitl_launch', px4_sitl_path)
    px4_sitl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(px4_sitl)

    config_path = LaunchConfiguration('drone_spawn_config_file').perform(context)
    config_dir = Path(config_path).resolve().parent
    with open(config_path) as f:
        config = json.load(f)

    # Apply PX4-specific SDF transforms to each model before spawning.
    for model in config.get('sdf_models', []):
        path = model.get('path')
        if path:
            sdf_path = resolve_resource_path(path, config_dir)
            sdf_path = px4_sitl.create_sdf_without_multicopter_velocity_control(sdf_path)
            sdf_path = px4_sitl.create_sdf_with_px4_sim_sensors(sdf_path)
            sdf_path = px4_sitl.create_sdf_with_px4_motor_joint_names(sdf_path)
            sdf_path = px4_sitl.create_model_sdf_with_harmonic_plugins(sdf_path)
            model['path'] = sdf_path

    temp_config = tempfile.NamedTemporaryFile(
        mode='w', prefix='waywiser_px4_spawn_', suffix='.json', delete=False
    )
    with temp_config:
        json.dump(config, temp_config)

    return [
        TimerAction(
            period=float(LaunchConfiguration('drone_spawn_delay').perform(context)),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            os.path.join(
                                waywiser_gazebo_dir,
                                'launch',
                                'spawn.launch.py',
                            )
                        ]
                    ),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                        'world': LaunchConfiguration('world').perform(context),
                        'spawn_config_file': temp_config.name,
                        'start_gazebo_bridge': 'False',
                        'spawn_backend': 'gz_service',
                        'gz_service_timeout': LaunchConfiguration(
                            'drone_spawn_service_timeout'
                        ).perform(context),
                        'gz_service_suppress_output': 'True',
                    }.items(),
                )
            ],
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

    return create_drone_vehicle_node(
        context,
        node_name=LaunchConfiguration('control_vehicle_node_name'),
        parameter_overrides={
            'enable_autopilot_component': enable_autopilot,
            'enable_px4_bridge': True,
        },
        remappings=[
            ('/cmd_vel_in', 'twist_safety_vel'),
            ('/cmd_vel_out', 'cmd_vel_out'),
        ],
    )


def drone_waypoint_follower_launch(context):
    return create_drone_vehicle_node(
        context,
        node_name='waywiser_drone_waypoint_follower',
        parameter_overrides={
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
            'joint_states_topic': '',
            'min_steering_height': 0.0,
            'publish_odom_to_baselink_tf': False,
            'publish_world_to_odom_tf': False,
        },
        remappings=[
            ('/cmd_vel_in', 'waypoint_follower_cmd_vel_in'),
            ('/cmd_vel_out', 'teleop_mux_vel'),
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
    from waywiser_description_py.waywiser_description_utils import get_scaled_urdf_string

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
