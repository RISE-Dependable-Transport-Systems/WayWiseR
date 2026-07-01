import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    waywiser_perception_dir = get_package_share_directory('waywiser_perception')

    # args that can be set from the command line or a default will be used
    collision_monitor_config_la = DeclareLaunchArgument(
        'collision_monitor_config',
        default_value=os.path.join(waywiser_perception_dir, 'config/collision_monitor.yaml'),
        description='Full path to params file for CollisionMonitorNode node.',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    # start nodes and use args to set parameters

    waywiser_collision_monitor_launch_action = OpaqueFunction(
        function=waywiser_collision_monitor_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(collision_monitor_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(waywiser_collision_monitor_launch_action)

    return ld


def waywiser_collision_monitor_launch(context):
    collision_monitor_config = FileUtils.get_full_file_path(
        LaunchConfiguration('collision_monitor_config').perform(context)
    )
    node_params_dict = RosUtils.get_node_params(
        collision_monitor_config, 'waywiser_collision_monitor_node'
    )

    waywiser_collision_monitor = Node(
        package='waywiser_perception',
        executable='collision_monitor_node.py',
        name='waywiser_collision_monitor_node',
        parameters=[
            node_params_dict,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        emulate_tty=True,
    )

    return [waywiser_collision_monitor]
