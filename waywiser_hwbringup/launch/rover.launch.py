# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.descriptions import ParameterValue
import yaml


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory('waywiser_hwbringup')
    description_dir = get_package_share_directory('waywiser_description')

    # args that can be set from the command line or a default will be used
    rover_config_la = DeclareLaunchArgument(
        'rover_config',
        default_value=os.path.join(hw_bringup_dir, 'config/rover.yaml'),
        description='Full path to params file of rover',
    )

    lidar_config_la = DeclareLaunchArgument(
        'lidar_config',
        default_value=os.path.join(hw_bringup_dir, 'config/lidar.yaml'),
        description='Full path to params file of lidar',
    )

    camera_config_la = DeclareLaunchArgument(
        'camera_config',
        default_value=os.path.join(hw_bringup_dir, 'config/realsense_d435i_camera.yaml'),
        description='Full path to params file of camera',
    )

    robot_state_publisher_la = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_dir, 'urdf/robot.urdf.xacro'),
        description='Full path to robot urdf file',
    )

    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )

    # start nodes and use args to set parameters
    waywise_node = Node(
        package='waywiser_node',
        executable='waywise_rover',
        name='waywise_rover_node',
        parameters=[LaunchConfiguration('rover_config')],
        remappings=[('/cmd_vel', '/cmd_vel_out')],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('model'), ' sim_mode:=', 'False']),
                    value_type=str,
                ),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
            }
        ],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    lidar_conditional_launch_action = OpaqueFunction(function=lidar_conditional_launch)
    camera_conditional_launch_action = OpaqueFunction(function=camera_conditional_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(rover_config_la)
    ld.add_action(lidar_config_la)
    ld.add_action(camera_config_la)
    ld.add_action(robot_state_publisher_la)
    ld.add_action(frame_prefix_la)

    # start nodes
    ld.add_action(waywise_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(lidar_conditional_launch_action)
    ld.add_action(camera_conditional_launch_action)

    return ld


def lidar_conditional_launch(context):
    enable_lidar = False
    with open(LaunchConfiguration('rover_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        waywise_rover_node_params_dict = config_data['waywise_rover_node']['ros__parameters']
        if 'enable_lidar' in waywise_rover_node_params_dict:
            enable_lidar = waywise_rover_node_params_dict['enable_lidar']

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

    with open(LaunchConfiguration('rover_config').perform(context)) as f:
        config_data = yaml.safe_load(f)
        waywise_rover_node_params_dict = config_data['waywise_rover_node']['ros__parameters']
        if 'enable_camera' in waywise_rover_node_params_dict:
            enable_camera = waywise_rover_node_params_dict['enable_camera']

    camera_params_dict = {}
    # Fix frame_id for depth camera point cloud: https://github.com/IntelRealSense/realsense-ros/tree/ros2-development?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems # noqa
    enable_pointcloud_tranformation = False
    pointcloud_tranformation_params_dict = {}
    if enable_camera:
        with open(LaunchConfiguration('camera_config').perform(context)) as f:
            camera_params_dict = yaml.safe_load(f)
            if 'pointcloud_tranformation' in camera_params_dict:
                pointcloud_tranformation_params_dict = camera_params_dict[
                    'pointcloud_tranformation'
                ]
                if 'enable' in pointcloud_tranformation_params_dict:
                    enable_pointcloud_tranformation = pointcloud_tranformation_params_dict[
                        'enable'
                    ]

    camera_container = GroupAction(
        actions=[
            PushRosNamespace('/sensors/camera'),
            ComposableNodeContainer(
                name='camera_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[{'autostart': 'True'}],
                arguments=['--ros-args', '--log-level', 'info'],
                condition=IfCondition(str(enable_camera)),
                output='screen',
                composable_node_descriptions=[
                    ComposableNode(
                        package='realsense2_camera',
                        plugin='realsense2_camera::RealSenseNodeFactory',
                        name='camera_node',
                        parameters=[camera_params_dict],
                        remappings=[
                            (
                                '/sensors/camera/depth/color/points',
                                '/sensors/camera/depth/points_raw',
                            ),
                        ],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='waywiser_hwbringup',
                        plugin='waywiser_hwbringup::PointCloudTransformer',
                        name='point_cloud_transformer_node',
                        condition=IfCondition(str(enable_pointcloud_tranformation)),
                        parameters=[pointcloud_tranformation_params_dict],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ],
            ),
        ]
    )

    return [camera_container]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
