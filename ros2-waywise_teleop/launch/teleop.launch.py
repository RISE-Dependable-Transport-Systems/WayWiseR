import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_dir = get_package_share_directory("ros2-waywise_teleop")

    # args that can be set from the command line or a default will be used
    joy_la = DeclareLaunchArgument(
        "joy_config",
        default_value=os.path.join(teleop_dir, "config/joy_teleop.yaml"),
        description="Full path to params file",
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
        remappings={("/cmd_vel", "/cmd_vel_out")},
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(joy_la)

    # start nodes
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)

    return ld
