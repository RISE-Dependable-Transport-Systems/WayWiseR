import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from waywiser_py.waywiser_utils import get_full_file_path


def generate_launch_description():
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    waywiser_hwbringup_dir = get_package_share_directory('waywiser_hwbringup')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')

    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/rover.yaml'),
        description='Full path to params file of rover',
    )
    lidar_config_la = DeclareLaunchArgument(
        'lidar_config',
        default_value=os.path.join(waywiser_hwbringup_dir, 'config/lidar.yaml'),
        description='Full path to params file of lidar',
    )
    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='False',
        description='Use Nav2 collision monitoring',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value='odom_reference_frame_rover.rviz',
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
            'vehicle_config': LaunchConfiguration('vehicle_config'),
            'frame_prefix': LaunchConfiguration('frame_prefix'),
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('waywiser_core'),
                    'launch',
                    'waywiser_localization.launch.py',
                )
            ]
        ),
        launch_arguments={
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
            'enable_collision_monitor': LaunchConfiguration('enable_collision_monitor'),
            'twist_safety_config': LaunchConfiguration('vehicle_config'),
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
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
            'control_vehicle_node': LaunchConfiguration('control_vehicle_node'),
        }.items(),
    )

    # create opaque functions to launch nodes using context
    lidar_conditional_launch_action = OpaqueFunction(function=lidar_conditional_launch)
    camera_conditional_launch_action = OpaqueFunction(function=camera_conditional_launch)
    slam_conditional_launch_action = OpaqueFunction(function=slam_conditional_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(vehicle_config_la)
    ld.add_action(lidar_config_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(enable_collision_monitor_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(control_vehicle_node_name_la)
    ld.add_action(localization_node_name_la)
    ld.add_action(lidar_based_slam_la)
    ld.add_action(slam_config_la)

    # start nodes
    ld.add_action(waywiser_car_launch)
    ld.add_action(localization_launch)
    ld.add_action(twist_safety)
    ld.add_action(teleop_rviz2)
    ld.add_action(lidar_conditional_launch_action)
    ld.add_action(camera_conditional_launch_action)
    ld.add_action(slam_conditional_launch_action)

    return ld


def lidar_conditional_launch(context):
    enable_lidar = False
    vehicle_config = get_full_file_path(LaunchConfiguration('vehicle_config').perform(context))
    if vehicle_config == '':
        return []

    with open(vehicle_config, 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f)
        waywise_car_node_params_dict = config_data['waywiser_car_node']['ros__parameters']
        if 'enable_lidar' in waywise_car_node_params_dict:
            enable_lidar = waywise_car_node_params_dict['enable_lidar']

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[LaunchConfiguration('lidar_config')],
        output='screen',
        condition=IfCondition(str(enable_lidar)),
        remappings=[('/scan', '/scan_lidar')],
    )

    return [lidar_node]


def camera_conditional_launch(context):
    enable_camera = False
    vehicle_config = get_full_file_path(LaunchConfiguration('vehicle_config').perform(context))
    if vehicle_config == '':
        return []

    with open(vehicle_config, 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f)
        waywise_car_node_params_dict = config_data['waywiser_car_node']['ros__parameters']
        if 'enable_camera' in waywise_car_node_params_dict:
            enable_camera = waywise_car_node_params_dict['enable_camera']

    camera_launch_acton = []
    if enable_camera:
        camera_launch_acton = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory('waywiser_hwbringup'),
                            'launch',
                            'realsense_d435i.launch.py',
                        )
                    ]
                ),
            )
        ]

    return camera_launch_acton


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
                }.items(),
            )
        ]

    return []


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r', encoding='utf-8') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
