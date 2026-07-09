import json
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap

from waywiser_description_py.waywiser_description_utils import (
    get_robot_state_publisher_node,
)
import yaml


def generate_launch_description():
    waywiser_dir = get_package_share_directory('waywiser')
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')
    waywiser_core_dir = get_package_share_directory('waywiser_core')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    gazebo_world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/car_world.sdf'),
        description='Full path to gazebo sdf file',
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
    rover_enable_collision_monitor_la = DeclareLaunchArgument(
        'rover_enable_collision_monitor',
        default_value='False',
        description='Use Nav2 collision monitoring',
    )
    rover_config_la = DeclareLaunchArgument(
        'rover_config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/rover.yaml'),
        description='Full path to params file of rover',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            waywiser_rviz2_dir, 'config/map_reference_frame_rover_drone_collab.rviz'
        ),
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
    control_vehicle_node_name_la = DeclareLaunchArgument(
        'control_vehicle_node_name',
        default_value='waywiser_car_node',
        description='Name of the vehicle node to control',
    )
    rover_localization_node_name_la = DeclareLaunchArgument(
        'rover_localization_node_name',
        default_value='waywiser_car_localization_node',
        description='Name of the node to be launched',
    )
    rover_lidar_based_slam_la = DeclareLaunchArgument(
        'rover_lidar_based_slam',
        default_value='True',
        description='Use lidar based slam',
    )
    rover_slam_config_la = DeclareLaunchArgument(
        'rover_slam_config',
        default_value=os.path.join(waywiser_slam_dir, 'config/slam.yaml'),
        description='Full path to params file for slam toolbox',
    )
    rover_spawn_config_file_la = DeclareLaunchArgument(
        'rover_spawn_config_file',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/rover_spawn_config.json'),
        description='Full path to spawn config file',
    )
    rover_yolo_config_la = DeclareLaunchArgument(
        'rover_yolo_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/yolov8.yaml'),
        description='Full path to params file of yolo',
    )
    rover_octomap_config_la = DeclareLaunchArgument(
        'rover_octomap_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/octomap.yaml'),
        description='Full path to params file of octomap',
    )
    rover_name_la = DeclareLaunchArgument(
        'rover_name',
        default_value='rover',
        description='Name of the rover, used as ROS namespace and prefix for robot frames',
    )
    drone_config_la = DeclareLaunchArgument(
        'drone_config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/drone.yaml'),
        description='Full path to params file of drone',
    )
    drone_spawn_config_file_la = DeclareLaunchArgument(
        'drone_spawn_config_file',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/static_drone_spawn_config.json'),
        description='Full path to spawn config file',
    )
    drone_yolo_config_la = DeclareLaunchArgument(
        'drone_yolo_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/yolov8.yaml'),
        description='Full path to params file of yolo',
    )
    drone_octomap_config_la = DeclareLaunchArgument(
        'drone_octomap_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/octomap.yaml'),
        description='Full path to params file of octomap',
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

    startup_route_file_la = DeclareLaunchArgument(
        'startup_route_file',
        default_value='',
        description='Route file to load in Control Tower at startup',
    )

    drone_name = LaunchConfiguration('drone_name')
    drone_frame_prefix = [drone_name, '/']

    # include launch files
    gazebo_rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_dir,
                    'launch',
                    'gazebo_rover.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'launch_gazebo_orchestrator': LaunchConfiguration('launch_gazebo_orchestrator'),
            'gazebo_orchestrator_config': LaunchConfiguration('gazebo_orchestrator_config'),
            'rover_enable_nav2_collision_monitor': LaunchConfiguration(
                'rover_enable_collision_monitor'
            ),
            'rover_config': LaunchConfiguration('rover_config'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
            'control_vehicle_node_name': LaunchConfiguration('control_vehicle_node_name'),
            'rover_localization_node_name': LaunchConfiguration('rover_localization_node_name'),
            'rover_lidar_based_slam': LaunchConfiguration('rover_lidar_based_slam'),
            'rover_slam_config': LaunchConfiguration('rover_slam_config'),
            'rover_spawn_config_file': LaunchConfiguration('rover_spawn_config_file'),
            'rover_yolo_config': LaunchConfiguration('rover_yolo_config'),
            'rover_octomap_config': LaunchConfiguration('rover_octomap_config'),
            'rover_name': LaunchConfiguration('rover_name'),
            'vehicle_control_enabled': LaunchConfiguration('vehicle_control_enabled'),
            'publish_control_tower_heartbeat': LaunchConfiguration(
                'publish_control_tower_heartbeat'
            ),
            'map_source': LaunchConfiguration('map_source'),
            'startup_route_file': LaunchConfiguration('startup_route_file'),
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
                            get_package_share_directory('waywiser_core'),
                            'launch',
                            'navsatfix_extended_wrapper.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'config': LaunchConfiguration('drone_config'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_prefix': drone_frame_prefix,
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
                    'frame_prefix': drone_frame_prefix,
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

    drone_octomap = GroupAction(
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
                            'octomap.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'octomap_config': LaunchConfiguration('drone_octomap_config'),
                }.items(),
            ),
        ]
    )

    drone_gazebo_spawn = OpaqueFunction(function=drone_spawn_request_launch)

    drone_state_publisher = OpaqueFunction(function=drone_state_publisher_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_world_la)
    ld.add_action(launch_gazebo_orchestrator_la)
    ld.add_action(gazebo_orchestrator_config_la)
    ld.add_action(rover_enable_collision_monitor_la)
    ld.add_action(rover_config_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(rover_localization_node_name_la)
    ld.add_action(rover_lidar_based_slam_la)
    ld.add_action(rover_slam_config_la)
    ld.add_action(rover_spawn_config_file_la)
    ld.add_action(rover_yolo_config_la)
    ld.add_action(rover_octomap_config_la)
    ld.add_action(rover_name_la)
    ld.add_action(drone_config_la)
    ld.add_action(drone_spawn_config_file_la)
    ld.add_action(drone_yolo_config_la)
    ld.add_action(drone_octomap_config_la)
    ld.add_action(drone_name_la)
    ld.add_action(drone_localization_node_name_la)
    ld.add_action(vehicle_control_enabled_la)
    ld.add_action(publish_control_tower_heartbeat_la)
    ld.add_action(map_source_la)
    ld.add_action(startup_route_file_la)

    # start nodes
    ld.add_action(gazebo_rover)
    ld.add_action(drone_navsatfix_extended_wrapper)
    ld.add_action(drone_gazebo_spawn)
    ld.add_action(drone_yolo)
    ld.add_action(drone_localization)
    ld.add_action(drone_octomap)
    ld.add_action(drone_state_publisher)

    return ld


def drone_state_publisher_launch(context):
    drone_config = LaunchConfiguration('drone_config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in [
        'true',
        '1',
        'yes',
    ]

    drone_name = LaunchConfiguration('drone_name').perform(context)
    drone_frame_prefix = drone_name + '/'

    # Robot State Publisher for drone
    config_data = yaml_to_dict(drone_config)
    rsp_params = config_data['/**']['ros__parameters']
    rsp_node = get_robot_state_publisher_node(
        context, rsp_params, use_sim_time, drone_frame_prefix
    )

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


def drone_spawn_request_launch(context):
    request = {
        'reset_gazebo': False,
        'spawn_models': True,
        'spawn_config_file': LaunchConfiguration('drone_spawn_config_file').perform(context),
        'spawn_interval_sec': 1.0,
        'spawn_backend': 'ros_gz_sim',
    }
    return [
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'topic',
                        'pub',
                        '--once',
                        '/setup_request',
                        'std_msgs/msg/String',
                        f"{{data: '{json.dumps(request)}'}}",
                    ],
                    output='log',
                )
            ],
        )
    ]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r', encoding='utf-8') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
