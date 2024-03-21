import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    waywiser_nav2_dir = get_package_share_directory('waywiser_nav2')

    # args that can be set from the command line or a default will be used
    nav2_path_follower_config_la = DeclareLaunchArgument(
        'nav2_path_follower_config',
        default_value=os.path.join(waywiser_nav2_dir, 'config/nav2.yaml'),
        description='Full path to params file for nav2 path follower',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    enable_velocity_smoother_la = DeclareLaunchArgument(
        'enable_velocity_smoother',
        default_value='True',
        description='Use Nav2 velocity smoother',
    )

    # start nodes and use args to set parameters

    conditional_launch_action = OpaqueFunction(function=conditional_launch_setup)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(nav2_path_follower_config_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(enable_velocity_smoother_la)

    # start nodes
    ld.add_action(conditional_launch_action)

    return ld


def conditional_launch_setup(context):
    enable_velocity_smoother = (
        LaunchConfiguration('enable_velocity_smoother').perform(context)
    ).lower() == 'true'

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/bond', '/bond_nav2_path_follower'),
    ]
    use_respawn = False

    param_substitutions = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('nav2_path_follower_config'),
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        'controller_server',
        'behavior_server',
    ]
    if enable_velocity_smoother:
        lifecycle_nodes.append('velocity_smoother')
        nav2_path_follower_nodes = GroupAction(
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'nav2_vel_unsmoothed')],
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'nav2_vel_unsmoothed')],
                ),
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings
                    + [('cmd_vel', 'nav2_vel_unsmoothed'), ('cmd_vel_smoothed', 'nav2_vel')],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_nav2_path_follower',
                    output='screen',
                    emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                    parameters=[
                        {
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'autostart': True,
                            'node_names': lifecycle_nodes,
                            'bond_timeout': 0.0,
                        }
                    ],
                    remappings=remappings,
                ),
            ],
        )
    else:
        nav2_path_follower_nodes = GroupAction(
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'nav2_vel')],
                ),
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'nav2_vel')],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_nav2_path_follower',
                    output='screen',
                    emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                    parameters=[
                        {
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'autostart': True,
                            'node_names': lifecycle_nodes,
                            'bond_timeout': 0.0,
                        }
                    ],
                    remappings=remappings,
                ),
            ],
        )

    return [nav2_path_follower_nodes]
