import importlib.util
import json
import os
import socket
import tempfile

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
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

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

    # Zenoh router connection settings (read from environment / .env)
    zenoh_remote_ip = os.environ.get('ZENOH_REMOTE_ROUTER_IP', '127.0.0.1') or '127.0.0.1'
    zenoh_remote_port = os.environ.get('ZENOH_REMOTE_ROUTER_PORT', '7447') or '7447'

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    gazebo_world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/bounded_world.sdf'),
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
        default_value='1.0',
        description='Delay (seconds) before spawning drone models to let Gazebo world initialize.',
    )
    px4_start_delay_la = DeclareLaunchArgument(
        'px4_start_delay',
        default_value='2.0',
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
        }.items(),
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
    drone_vehicle_node = OpaqueFunction(function=drone_vehicle_node_launch)
    zenoh_router = OpaqueFunction(function=zenoh_router_check)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_world_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(drone_config_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(drone_spawn_config_file_la)
    ld.add_action(drone_yolo_config_la)
    ld.add_action(drone_name_la)
    ld.add_action(drone_localization_node_name_la)
    ld.add_action(px4_sys_autostart_la)
    ld.add_action(drone_spawn_delay_la)
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
                px4_sitl,
                drone_gazebo_spawn,
                drone_twist_safety,
                teleop_rviz2,
                drone_navsatfix_extended_wrapper,
                drone_yolo,
                drone_localization,
                drone_rgbd_to_pointcloud,
                drone_state_publisher,
                drone_vehicle_node,
            ],
        )
    )

    return ld


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
    # The get_robot_state_publisher_node function updates rsp_params['urdf_file'] in-place
    # with the scaled URDF string. We can reuse this for the vehicle node.
    urdf_string = rsp_params['urdf_file']

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


def drone_gazebo_spawn_launch(context):
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')

    # Load PX4 SDF transform utilities from px4_sitl.launch.py.
    px4_sitl_path = os.path.join(waywiser_gazebo_dir, 'launch', 'px4_sitl.launch.py')
    spec = importlib.util.spec_from_file_location('px4_sitl_launch', px4_sitl_path)
    px4_sitl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(px4_sitl)

    config_path = LaunchConfiguration('drone_spawn_config_file').perform(context)
    with open(config_path) as f:
        config = json.load(f)

    # Apply PX4-specific SDF transforms to each model before spawning.
    for model in config.get('sdf_models', []):
        path = model.get('path')
        if path:
            sdf_path = path
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
    node_params = {**shared_params, **node_specific_params}

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
                    name=LaunchConfiguration('control_vehicle_node_name'),
                    parameters=[
                        node_params,
                        {
                            'use_sim_time': use_sim_time,
                            'frame_prefix': f'{drone_name}/',
                            'urdf_file': urdf_string,
                        },
                    ],
                    remappings=[
                        ('/cmd_vel_in', 'twist_safety_vel'),
                        ('/cmd_vel_out', 'cmd_vel_out'),
                    ],
                    output='screen',
                ),
            ]
        )
    ]
