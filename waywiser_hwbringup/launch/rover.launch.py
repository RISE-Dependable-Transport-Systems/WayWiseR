# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[yaml_to_dict(LaunchConfiguration('camera_config').perform(context))],
        output='screen',
        condition=IfCondition(str(enable_camera)),
        arguments=['--ros-args', '-r', '__ns:=/sensors'],
        remappings=[
            (
                '/sensors/camera/depth/color/points',
                '/sensors/camera/depth/points',
            )
        ],
    )

    return [camera_node]


def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)
