import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    waywiser_nav2_dir = get_package_share_directory('waywiser_nav2')

    # args that can be set from the command line or a default will be used
    nav2_waypoint_navigator_config_la = DeclareLaunchArgument(
        'nav2_waypoint_navigator_config',
        default_value=os.path.join(waywiser_nav2_dir, 'config/nav2.yaml'),
        description='Full path to params file for nav2 waypoint navigator',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    # start nodes and use args to set parameters
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/bond', '/bond_nav2_waypoint_navigator'),
    ]
    use_respawn = False

    param_substitutions = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('nav2_waypoint_navigator_config'),
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    nav2_waypoint_navigator_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_nav2_path_planner',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'autostart': True,
                        'node_names': ['waypoint_navigator'],
                        'bond_timeout': 0.0,
                    }
                ],
                remappings=remappings,
            ),
        ],
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(nav2_waypoint_navigator_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(nav2_waypoint_navigator_nodes)

    return ld
