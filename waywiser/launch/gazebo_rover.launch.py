import json
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')
    default_gazebo_osm_tile_cache_root = os.path.join(
        os.environ.get('WAYWISER_WS', os.getcwd()),
        'resources',
        'control_tower',
        'gazebo',
    )

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    simulator_nodes_la = DeclareLaunchArgument(
        'simulator_nodes',
        default_value='True',
        description='Launch Gazebo and simulator-side rover support nodes',
    )
    gazebo_world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/bounded_world.sdf'),
        description='Full path to gazebo sdf file',
    )
    rover_enable_nav2_collision_monitor_la = DeclareLaunchArgument(
        'rover_enable_nav2_collision_monitor',
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
        default_value=os.path.join(waywiser_rviz2_dir, 'config/map_reference_frame_rover.rviz'),
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
        default_value='',
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
    control_vehicle_node_name_la = DeclareLaunchArgument(
        'control_vehicle_node_name',
        default_value='waywiser_car_node',
        description='Name of the vehicle node to control',
    )
    rover_vehicle_node_la = DeclareLaunchArgument(
        'rover_vehicle_node',
        default_value='True',
        description='Launch the main WayWiseR rover vehicle node',
    )
    rover_vehicle_node_enable_autopilot_la = DeclareLaunchArgument(
        'rover_vehicle_node_enable_autopilot',
        default_value='True',
        description='Enable the waypoint follower inside the main rover vehicle node',
    )
    rover_waypoint_follower_la = DeclareLaunchArgument(
        'rover_waypoint_follower',
        default_value='False',
        description='Launch a command-side rover waypoint follower that publishes velocity commands',
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

    rover_name = LaunchConfiguration('rover_name')
    frame_prefix = [rover_name, '/']

    # include launch files
    waywiser_car_launch = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_core_dir,
                            'launch',
                            'waywiser_car.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'vehicle_config': LaunchConfiguration('rover_config'),
                    'frame_prefix': frame_prefix,
                    'node_name': LaunchConfiguration('control_vehicle_node_name'),
                    'enable_autopilot_component': LaunchConfiguration(
                        'rover_vehicle_node_enable_autopilot'
                    ),
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('rover_vehicle_node')),
    )

    rover_waypoint_follower = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_core_dir,
                            'launch',
                            'waywiser_car.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'vehicle_config': LaunchConfiguration('rover_config'),
                    'frame_prefix': frame_prefix,
                    'node_name': 'waywiser_car_waypoint_follower_node',
                    'enable_autopilot_component': 'True',
                    'publish_world_to_odom_tf': 'False',
                    'cmd_vel_in': 'waypoint_follower_cmd_vel_in',
                    'cmd_vel_out': 'waypoint_follower_vel',
                    'publish_joint_state_publisher': 'False',
                    'parameter_overrides_json': json.dumps(
                        {
                            'enable_mavlink_interface': False,
                            'control_tower_selectable': False,
                            'vehicle_control_command_topic': 'waypoint_follower_control_cmd',
                            'vehicle_pose_topic': 'waypoint_follower_pose',
                            'control_tower_heartbeat_topic': '',
                            'control_tower_heartbeat_rx_state_topic': '',
                            'emergency_stop_update_topic': (
                                'waypoint_follower_emergency_stop/target_state'
                            ),
                            'joint_states_topic': 'waypoint_follower_joint_states',
                        }
                    ),
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('rover_waypoint_follower')),
    )

    rover_navsatfix_extended_wrapper = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
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
                    'config': LaunchConfiguration('rover_config'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    rover_localization = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
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
                    'localization_config': LaunchConfiguration('rover_config'),
                    'localization_node_name': LaunchConfiguration('rover_localization_node_name'),
                    'frame_prefix': frame_prefix,
                }.items(),
            ),
        ]
    )

    rover_twist_safety = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
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
                    'enable_nav2_collision_monitor': LaunchConfiguration(
                        'rover_enable_nav2_collision_monitor'
                    ),
                    'twist_safety_config': LaunchConfiguration('rover_config'),
                }.items(),
            ),
        ]
    )

    rover_yolo = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
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
                    'yolo_config': LaunchConfiguration('rover_yolo_config'),
                }.items(),
            ),
        ]
    )

    rover_octomap = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
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
                    'octomap_config': LaunchConfiguration('rover_octomap_config'),
                }.items(),
            ),
        ]
    )

    rover_lidar_based_slam = GroupAction(
        actions=[
            PushRosNamespace(rover_name),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            waywiser_slam_dir,
                            'launch',
                            'slam.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam_config': LaunchConfiguration('rover_slam_config'),
                    'frame_prefix': frame_prefix,
                }.items(),
                condition=IfCondition(LaunchConfiguration('rover_lidar_based_slam')),
            ),
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_gazebo_dir,
                    'launch',
                    'gazebo_orchestrator.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'use_nvidia_gpu': LaunchConfiguration('use_nvidia_gpu'),
            'launch_gazebo_orchestrator': LaunchConfiguration('launch_gazebo_orchestrator'),
            'gazebo_orchestrator_config': LaunchConfiguration('gazebo_orchestrator_config'),
            'spawn_config_file': LaunchConfiguration('rover_spawn_config_file'),
            'spawn_on_startup': 'True',
            'spawn_backend': 'ros_gz_sim',
        }.items(),
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
                rover_name,
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

    gazebo_osm_tile_server = Node(
        package='waywiser_gazebo',
        executable='gazebo_osm_tile_server_node.py',
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

    simulator_nodes = GroupAction(
        actions=[
            gazebo,
            gazebo_osm_tile_server,
            rover_twist_safety,
            waywiser_car_launch,
            rover_navsatfix_extended_wrapper,
            rover_localization,
            rover_lidar_based_slam,
            rover_yolo,
            rover_octomap,
        ],
        condition=IfCondition(LaunchConfiguration('simulator_nodes')),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(simulator_nodes_la)
    ld.add_action(gazebo_world_la)
    ld.add_action(rover_enable_nav2_collision_monitor_la)
    ld.add_action(rover_config_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(vehicle_control_enabled_la)
    ld.add_action(publish_control_tower_heartbeat_la)
    ld.add_action(map_source_la)
    ld.add_action(gazebo_osm_tile_server_la)
    ld.add_action(gazebo_osm_tile_server_config_la)
    ld.add_action(launch_gazebo_orchestrator_la)
    ld.add_action(gazebo_orchestrator_config_la)
    ld.add_action(gazebo_osm_tile_server_url_la)
    ld.add_action(gazebo_osm_tile_cache_dir_la)
    ld.add_action(startup_route_file_la)
    ld.add_action(use_nvidia_gpu_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(rover_vehicle_node_la)
    ld.add_action(rover_vehicle_node_enable_autopilot_la)
    ld.add_action(rover_waypoint_follower_la)
    ld.add_action(rover_localization_node_name_la)
    ld.add_action(rover_lidar_based_slam_la)
    ld.add_action(rover_slam_config_la)
    ld.add_action(rover_spawn_config_file_la)
    ld.add_action(rover_yolo_config_la)
    ld.add_action(rover_octomap_config_la)
    ld.add_action(rover_name_la)

    # start nodes
    ld.add_action(simulator_nodes)
    ld.add_action(teleop_rviz2)
    ld.add_action(rover_waypoint_follower)

    return ld
