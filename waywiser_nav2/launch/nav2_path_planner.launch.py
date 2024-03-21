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
    nav2_path_planner_config_la = DeclareLaunchArgument(
        'nav2_path_planner_config',
        default_value=os.path.join(waywiser_nav2_dir, 'config/nav2.yaml'),
        description='Full path to params file for nav2 path planner',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )

    enable_smoother_server_la = DeclareLaunchArgument(
        'enable_smoother_server',
        default_value='False',
        description='Use Nav2 smoother server',
    )

    # start nodes and use args to set parameters

    conditional_launch_action = OpaqueFunction(function=conditional_launch_setup)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(nav2_path_planner_config_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(enable_smoother_server_la)

    # start nodes
    ld.add_action(conditional_launch_action)

    return ld


def conditional_launch_setup(context):
    enable_smoother_server = (
        LaunchConfiguration('enable_smoother_server').perform(context)
    ).lower() == 'true'

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/bond', '/bond_nav2_path_planner'),
    ]
    use_respawn = False

    param_substitutions = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('nav2_path_planner_config'),
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        'planner_server',
        'bt_navigator',
    ]
    if enable_smoother_server:
        lifecycle_nodes.append('smoother_server')
        nav2_path_planner_nodes = GroupAction(
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
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
                            'node_names': lifecycle_nodes,
                            'bond_timeout': 0.0,
                        }
                    ],
                    remappings=remappings
                    + [
                        (
                            '/lifecycle_manager_nav2_path_planner/is_active',
                            '/lifecycle_manager_navigation/is_active',
                        ),
                    ],
                ),
            ],
        )
    else:
        nav2_path_planner_nodes = GroupAction(
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params],
                    remappings=remappings,
                ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
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
                            'node_names': lifecycle_nodes,
                            'bond_timeout': 0.0,
                        }
                    ],
                    remappings=remappings
                    + [
                        (
                            '/lifecycle_manager_nav2_path_planner/is_active',
                            '/lifecycle_manager_navigation/is_active',
                        ),
                    ],
                ),
            ],
        )

    return [nav2_path_planner_nodes]
