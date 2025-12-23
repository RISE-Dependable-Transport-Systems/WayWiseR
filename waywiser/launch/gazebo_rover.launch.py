import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def generate_launch_description():
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    gazebo_world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/car_world.sdf'),
        description='Full path to gazebo sdf file',
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='True',
        description='Use Nav2 collision monitoring',
    )
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config/rover.yaml'),
        description='Full path to params file of vehicle',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            waywiser_rviz2_dir, 'config/map_reference_frame_rover_slam.rviz'
        ),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop_sim.yaml'),
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
        'control_vehicle_node',
        default_value='waywiser_car_node',
        description='Name of the vehicle node to control',
    )
    localization_node_name_la = DeclareLaunchArgument(
        'localization_node_name',
        default_value='waywiser_car_localization_node',
        description='Name of the node to be launched',
    )
    lidar_based_slam_la = DeclareLaunchArgument(
        'lidar_based_slam',
        default_value='True',
        description='Use lidar based slam',
    )
    slam_config_la = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(waywiser_slam_dir, 'config/slam.yaml'),
        description='Full path to params file for slam toolbox',
    )

    # include launch files
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

    waywiser_car_launch = IncludeLaunchDescription(
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
            'vehicle_config': LaunchConfiguration('vehicle_config'),
        }.items(),
    )

    waywiser_car_localization = IncludeLaunchDescription(
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
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
            'control_vehicle_node': LaunchConfiguration('control_vehicle_node'),
        }.items(),
    )

    # create opaque functions to launch nodes using context
    slam_conditional_launch_action = OpaqueFunction(function=slam_conditional_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_world_la)
    ld.add_action(enable_collision_monitor_la)
    ld.add_action(vehicle_config_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(localization_node_name_la)
    ld.add_action(lidar_based_slam_la)
    ld.add_action(slam_config_la)

    # start nodes
    ld.add_action(gazebo)
    ld.add_action(twist_safety)
    ld.add_action(teleop_rviz2)
    ld.add_action(waywiser_car_launch)
    ld.add_action(waywiser_car_localization)
    ld.add_action(slam_conditional_launch_action)

    return ld


def slam_conditional_launch(context):
    lidar_based_slam = LaunchConfiguration('lidar_based_slam').perform(context)
    if lidar_based_slam.lower() == 'true':
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_slam'),
                            'launch',
                            'slam.launch.py',
                        )
                    ]
                ),
                launch_arguments={
                    'slam_config': LaunchConfiguration('slam_config'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        ]

    return []


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r', encoding='utf-8') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
