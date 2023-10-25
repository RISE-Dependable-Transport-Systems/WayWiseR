import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    teleop_dir = get_package_share_directory("ros2-waywise_teleop")

    # args that can be set from the command line or a default will be used
    joy_la = DeclareLaunchArgument(
        "joy_config",
        default_value=os.path.join(teleop_dir, "config/joy_teleop.yaml"),
        description="Full path to params file",
    )

    twist_mux_la = DeclareLaunchArgument(
        "twist_mux_config",
        default_value=os.path.join(teleop_dir, "config/twist_mux.yaml"),
        description="Full path to params file",
    )

    enable_keyboard_la = DeclareLaunchArgument(
        "enable_keyboard",
        default_value="false",
        description="Enable keyboard",
    )

    # start nodes and use args to set parameters
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
        parameters=[LaunchConfiguration("joy_config")],
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[LaunchConfiguration("joy_config")],
        remappings={("/cmd_vel", "/joy_vel")},
    )

    teleop_twist_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard_node",
        output="screen",
        prefix="xterm -e",
        condition=IfCondition(LaunchConfiguration("enable_keyboard")),
        remappings={("/cmd_vel", "/key_vel")},
    )

    teleop_gateway_node = Node(
        package="ros2-waywise_teleop",
        executable="teleop_gateway",
        name="teleop_gateway",
        parameters=[LaunchConfiguration("joy_config")],
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[LaunchConfiguration("twist_mux_config")],
        remappings={("/cmd_vel_out", "/cmd_vel_mux")},
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(joy_la)
    ld.add_action(twist_mux_la)
    ld.add_action(enable_keyboard_la)

    # start nodes
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(teleop_gateway_node)
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(twist_mux_node)

    return ld
