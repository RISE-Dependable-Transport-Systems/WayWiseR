import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    hw_bringup_dir = get_package_share_directory("waywiser_hwbringup")

    # Declare a launch argument for the YAML config file
    urm14_config_arg = DeclareLaunchArgument(
        "urm14_config",
        default_value=os.path.join(hw_bringup_dir, "config", "urm14.yaml"),
        description="Full path to params file of URM14",
    )

    # Use LaunchConfiguration to retrieve the argument's value
    urm14_config = LaunchConfiguration("urm14_config")

    # Define the nodes and explicitly pass the config file as a parameter
    urm14_publisher = Node(
        package="waywiser_hwbringup",
        executable="urm14_publisher.py",
        name="urm14_publisher",
        parameters=[urm14_config],  # Now the node will actually use the parameter file
    )

    urm14_subscriber = Node(
        package="waywiser_hwbringup",
        executable="urm14_subscriber.py",
        name="urm14_subscriber",
        parameters=[urm14_config],  # Also pass to subscriber
    )

    # Create LaunchDescription and add actions
    ld = LaunchDescription()
    ld.add_action(urm14_config_arg)
    ld.add_action(urm14_publisher)
    ld.add_action(urm14_subscriber)

    return ld
