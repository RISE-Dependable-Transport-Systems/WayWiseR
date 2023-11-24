# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    hw_bringup_dir = get_package_share_directory("ros2-waywise_hwbringup")
    description_dir = get_package_share_directory("ros2-waywise_description")

    # args that can be set from the command line
    model = LaunchConfiguration("model")
    frame_prefix = LaunchConfiguration("frame_prefix")

    # args that can be set from the command line or a default will be used
    rover_config_la = DeclareLaunchArgument(
        "rover_config",
        default_value=os.path.join(hw_bringup_dir, "config/rover.yaml"),
        description="Full path to params file of rover",
    )

    robot_state_publisher_la = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(description_dir, "urdf/robot.urdf.xacro"),
        description="Full path to robot urdf file",
    )

    frame_prefix_la = DeclareLaunchArgument(
        "frame_prefix",
        default_value="/",
        description="Prefix to publish robot transforms in",
    )

    enable_lidar_la = DeclareLaunchArgument(
        "enable_lidar",
        default_value="false",
        description="switch to enable lidar node",
    )

    lidar_serial_port_la = DeclareLaunchArgument(
        "lidar_serial_port", default_value="/dev/ttyUSB0"
    )
    lidar_serial_baudrate_la = DeclareLaunchArgument(
        "lidar_serial_baudrate", default_value="256000"
    )
    lidar_frame_id_la = DeclareLaunchArgument(
        "lidar_frame_id", default_value="lidar_link"
    )
    lidar_inverted_la = DeclareLaunchArgument("lidar_inverted", default_value="false")
    lidar_angle_compensate_la = DeclareLaunchArgument(
        "lidar_angle_compensate", default_value="true"
    )

    # start nodes and use args to set parameters
    waywise_node = Node(
        package="ros2-waywise_node",
        executable="waywise_rover",
        name="waywise_rover_node",
        parameters=[LaunchConfiguration("rover_config")],
        remappings=[("/cmd_vel", "/cmd_vel_out")],
        arguments=["--ros-args", "--log-level", "info"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", model, " sim_mode:=", "False"]),
                "frame_prefix": frame_prefix,
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            {
                "serial_port": LaunchConfiguration("lidar_serial_port"),
                "serial_baudrate": LaunchConfiguration("lidar_serial_baudrate"),
                "frame_id": LaunchConfiguration("lidar_frame_id"),
                "inverted": LaunchConfiguration("lidar_inverted"),
                "angle_compensate": LaunchConfiguration("lidar_angle_compensate"),
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_lidar")),
        remappings=[("/scan", "/scan_lidar")],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(rover_config_la)
    ld.add_action(robot_state_publisher_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(enable_lidar_la)
    ld.add_action(lidar_serial_port_la)
    ld.add_action(lidar_serial_baudrate_la)
    ld.add_action(lidar_frame_id_la)
    ld.add_action(lidar_inverted_la)
    ld.add_action(lidar_angle_compensate_la)

    # start nodes
    ld.add_action(waywise_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(lidar_node)

    return ld
