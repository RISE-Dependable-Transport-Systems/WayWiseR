import ast
import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_dir = get_package_share_directory("waywiser_gazebo")

    # args that can be set from the command line or a default will be used
    gazebo_la = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(gazebo_dir, "worlds/car_world.sdf"),
        description="Full path to gazebo sdf file",
    )
    gazebo_bridge_la = DeclareLaunchArgument(
        "gazebo_bridge",
        default_value=os.path.join(gazebo_dir, "config/ros_gazebo_bridges.yaml"),
        description="Full path to gazebo bridge file",
    )
    use_sim_time_la = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation/Gazebo clock"
    )
    frame_prefix_la = DeclareLaunchArgument(
        "frame_prefix",
        default_value="/",
        description="Prefix to publish robot transforms in",
    )
    gz_sim_resource_paths_la = DeclareLaunchArgument(
        "gz_sim_resource_paths",
        default_value="",
        description="Paths to additional model resources as a list",
    )

    # start nodes and use args to set parameters
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
    )

    # gazebo bridge
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            ["config_file:=", LaunchConfiguration("gazebo_bridge")],
        ],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # include launch files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),
    )

    # create launch description
    ld = LaunchDescription()
    ld.add_action(gz_sim_resource_paths_la)
    ld.add_action(OpaqueFunction(function=set_gz_sim_resources_path))

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(gazebo_la)
    ld.add_action(gazebo_bridge_la)

    # run gazebo launch file
    ld.add_action(gazebo)

    # spawn robot in gazebo
    ld.add_action(spawn_robot)

    # setup gazebo bridge
    ld.add_action(ros_gz_bridge_node)

    return ld


def set_gz_sim_resources_path(context):
    # Fetch GZ_SIM_RESOURCE_PATH environment variable
    gz_sim_resources_path = set(
        str(os.environ.get("GZ_SIM_RESOURCE_PATH", "")).split(":")
    )
    gz_sim_resources_path = {x for x in gz_sim_resources_path if x}

    # Default waywiser-related list of paths to Gazebo resources
    default_gz_sim_resources_path = [
        os.path.join(get_package_share_directory("waywiser_description"), "sdf"),
        str(
            Path(get_package_share_directory("waywiser_description")).parent.absolute()
        ),
    ]
    gz_sim_resources_path.update(set(default_gz_sim_resources_path))

    input_gz_sim_resource_paths = LaunchConfiguration("gz_sim_resource_paths").perform(
        context
    )
    if input_gz_sim_resource_paths != "":
        input_gz_sim_resource_paths = ast.literal_eval(input_gz_sim_resource_paths)
        if isinstance(input_gz_sim_resource_paths, list):
            gz_sim_resources_path.update(set(input_gz_sim_resource_paths))

    # print('gz_sim_resources_path:{}', gz_sim_resources_path)
    gz_sim_resources_path_set_action = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", ":".join(gz_sim_resources_path)
    )
    return [gz_sim_resources_path_set_action]
