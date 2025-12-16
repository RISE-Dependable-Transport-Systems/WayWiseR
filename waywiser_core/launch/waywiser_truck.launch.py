from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_description_py.waywiser_description_utils import get_robot_state_publisher_node
from waywiser_py.waywiser_utils import get_full_file_path
import yaml


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value='',
        description='Full path to params file of waywiser_truck',
    )
    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    # start nodes and use args to set parameters
    waywiser_truck_node_launch_action = OpaqueFunction(function=waywiser_truck_node_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(vehicle_config_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(waywiser_truck_node_launch_action)

    return ld


def waywiser_truck_node_launch(context):
    nodes = []
    vehicle_config = get_full_file_path(LaunchConfiguration('vehicle_config').perform(context))
    if vehicle_config == '':
        print('vehicle_config is empty! Skipping launching waywiser_truck_node.')
        return nodes

    use_sim_time_raw = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_raw.lower() in ['true', '1', 'yes']
    with open(vehicle_config, 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f)
        node_params_dict = config_data['/**']['ros__parameters']
        node_params_dict.update(config_data['waywiser_truck_node']['ros__parameters'])

        if 'urdf_file' in node_params_dict:
            robot_state_publisher_node = get_robot_state_publisher_node(
                node_params_dict, use_sim_time
            )
            if robot_state_publisher_node is not None:
                nodes.append(robot_state_publisher_node)
            else:
                node_params_dict.pop('urdf_file')

        nodes.append(
            Node(
                package='waywiser_core',
                executable='waywiser_truck_node',
                name='waywiser_truck_node',
                parameters=[
                    node_params_dict,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=[('/cmd_vel', '/cmd_vel_out')],
                arguments=['--ros-args', '--log-level', 'info'],
                output='screen',
                emulate_tty=True,
                # prefix='xterm -e gdb -q -ex run --args',
            )
        )

        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='ego_veh_joint_state_publisher',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'source_list': ['/waywiser_joint_states'],
                    }
                ],
            )
        )

    return nodes
