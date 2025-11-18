# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from waywiser_py.waywiser_utils import get_full_file_path
import yaml


def generate_launch_description():
    waywiser_core_dir = get_package_share_directory('waywiser_core')
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')

    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value=os.path.join(hw_bringup_dir, 'config/rover.yaml'),
        description='Full path to params file of rover',
    )
    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )
    lidar_config_la = DeclareLaunchArgument(
        'lidar_config',
        default_value=os.path.join(hw_bringup_dir, 'config/lidar.yaml'),
        description='Full path to params file of lidar',
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

    # create opaque functions to launch nodes using context
    lidar_conditional_launch_action = OpaqueFunction(function=lidar_conditional_launch)
    camera_conditional_launch_action = OpaqueFunction(function=camera_conditional_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(vehicle_config_la)
    ld.add_action(lidar_config_la)
    ld.add_action(frame_prefix_la)

    # start nodes
    ld.add_action(waywiser_car_launch)
    ld.add_action(lidar_conditional_launch_action)
    ld.add_action(camera_conditional_launch_action)

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


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r', encoding='utf-8') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
