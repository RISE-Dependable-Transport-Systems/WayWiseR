import os
import yaml
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    # filepath to configuration file
    teleop_config_path = os.path.join(
        get_package_share_directory("ros2-waywise_teleop"),
        "config/teleop.yaml"
    )

    # retrieve a parameter from configuration file
    with open(teleop_config_path, 'r') as f:
        parameters = yaml.safe_load(f)
        use_keyboard = f'{parameters["teleop_twist_keyboard_node"]["ros__parameters"]["use_keyboard"]}'

    # args that can be set from the command line or a default will be used
    teleop_la = DeclareLaunchArgument(
        "teleop_config",
        default_value=teleop_config_path,
        description="Full path to params file",
    )

    # start nodes and use args to set parameters
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
        parameters=[LaunchConfiguration("teleop_config")],
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[LaunchConfiguration("teleop_config")],
        remappings={("/cmd_vel", "/joy_vel")},
    )

    teleop_twist_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        condition=IfCondition(use_keyboard),
        remappings={("/cmd_vel", "/key_vel")},
    )

    teleop_gateway_node = Node(
        package="ros2-waywise_teleop",
        executable="teleop_gateway",
        name="teleop_gateway",
        parameters=[LaunchConfiguration("teleop_config")],
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[LaunchConfiguration("teleop_config")],
        remappings={("/cmd_vel_out", "/cmd_vel_mux")},
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(teleop_la)

    # start nodes
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(teleop_gateway_node)
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(twist_mux_node)

    return ld
