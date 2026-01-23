import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_py.waywiser_utils import RosUtils


def generate_launch_description():
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')

    # args that can be set from the command line or a default will be used
    twist_safety_config_la = DeclareLaunchArgument(
        'twist_safety_config',
        default_value=os.path.join(waywiser_twist_safety_dir, 'config/twist_safety.yaml'),
        description='Full path to params file',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    enable_nav2_collision_monitor_la = DeclareLaunchArgument(
        'enable_nav2_collision_monitor',
        default_value='False',
        description='Use Nav2 collision monitoring',
    )

    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )

    conditional_launch_action = OpaqueFunction(function=conditional_launch_setup)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(twist_safety_config_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(enable_nav2_collision_monitor_la)
    ld.add_action(frame_prefix_la)

    # start nodes
    ld.add_action(conditional_launch_action)

    return ld


def conditional_launch_setup(context):
    config_file = LaunchConfiguration('twist_safety_config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    enable_nav2_collision_monitor = (
        LaunchConfiguration('enable_nav2_collision_monitor').perform(context)
    ).lower() == 'true'

    launch_actions = []

    # 1. onboard_twist_mux
    launch_actions.append(
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='onboard_twist_mux',
            parameters=[
                RosUtils.get_node_params(config_file, 'onboard_twist_mux'),
                {'use_sim_time': use_sim_time},
            ],
            remappings={('cmd_vel_out', 'onboard_mux_vel')},
        )
    )

    if enable_nav2_collision_monitor:
        node_params_dict = RosUtils.get_node_params(config_file, 'nav2_collision_monitor')
        frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
        base_frame_id = node_params_dict.get('base_frame_id', 'base_link')
        odom_frame_id = node_params_dict.get('odom_frame_id', 'odom')
        node_params_dict['base_frame_id'] = RosUtils.join_frame(frame_prefix, base_frame_id)
        node_params_dict['odom_frame_id'] = RosUtils.join_frame(frame_prefix, odom_frame_id)

        nav2_collision_monitor = GroupAction(
            actions=[
                Node(
                    package='nav2_collision_monitor',
                    executable='collision_monitor',
                    name='nav2_collision_monitor',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        node_params_dict,
                        {'use_sim_time': use_sim_time},
                    ],
                    remappings={('/bond', 'bond_nav2_collision_monitor')},
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_collision_monitor',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {
                            'use_sim_time': use_sim_time,
                            'autostart': True,
                            'node_names': ['nav2_collision_monitor'],
                            'bond_timeout': 0.0,
                        }
                    ],
                    remappings={('/bond', 'bond_nav2_collision_monitor')},
                ),
            ],
        )

        emergency_stop_monitor_node = Node(
            package='waywiser_twist_safety',
            executable='emergency_stop_monitor',
            name='emergency_stop_monitor',
            parameters=[
                RosUtils.get_node_params(config_file, 'emergency_stop_monitor'),
                {'use_sim_time': use_sim_time},
            ],
            remappings={
                ('cmd_vel_in', 'nav2_collision_monitor_vel'),
            },
        )
        launch_actions.extend([nav2_collision_monitor, emergency_stop_monitor_node])
    else:
        emergency_stop_monitor_node = Node(
            package='waywiser_twist_safety',
            executable='emergency_stop_monitor',
            name='emergency_stop_monitor',
            parameters=[
                RosUtils.get_node_params(config_file, 'emergency_stop_monitor'),
                {'use_sim_time': use_sim_time},
            ],
            remappings={
                ('cmd_vel_in', 'onboard_mux_vel'),
                ('cmd_vel_out', 'twist_safety_vel'),
            },
        )
        launch_actions.append(emergency_stop_monitor_node)

    return launch_actions
