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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    gazebo_world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/car_world.sdf'),
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
                }.items(),
            ),
        ]
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
                    'gazebo.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    rover_gazebo_spawn = IncludeLaunchDescription(
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
            'spawn_config_file': LaunchConfiguration('rover_spawn_config_file')
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
        }.items(),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_world_la)
    ld.add_action(rover_enable_nav2_collision_monitor_la)
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
    ld.add_action(rover_name_la)

    # start nodes
    ld.add_action(gazebo)
    ld.add_action(rover_twist_safety)
    ld.add_action(teleop_rviz2)
    ld.add_action(waywiser_car_launch)
    ld.add_action(rover_navsatfix_extended_wrapper)
    ld.add_action(rover_localization)
    ld.add_action(rover_lidar_based_slam)
    ld.add_action(rover_gazebo_spawn)
    ld.add_action(rover_yolo)

    return ld
