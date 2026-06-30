import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_description_py.waywiser_description_utils import (
    get_robot_state_publisher_node,
)
from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    vehicle_config_la = DeclareLaunchArgument(
        'vehicle_config',
        default_value='',
        description='Full path to params file of waywiser_car',
    )
    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )
    publish_map_to_odom_tf_la = DeclareLaunchArgument(
        'publish_world_to_odom_tf', default_value='True', description='Publish map to odom tf'
    )
    node_name_la = DeclareLaunchArgument(
        'node_name',
        default_value='waywiser_car_node',
        description='Name of the waywiser car node',
    )
    enable_autopilot_component_la = DeclareLaunchArgument(
        'enable_autopilot_component',
        default_value='',
        description='Optional override for the car node waypoint follower/autopilot component',
    )
    cmd_vel_in_la = DeclareLaunchArgument(
        'cmd_vel_in',
        default_value='twist_safety_vel',
        description='Topic remapped to /cmd_vel_in for the car node',
    )
    cmd_vel_out_la = DeclareLaunchArgument(
        'cmd_vel_out',
        default_value='cmd_vel_out',
        description='Topic remapped from /cmd_vel_out for the car node',
    )
    publish_joint_state_publisher_la = DeclareLaunchArgument(
        'publish_joint_state_publisher',
        default_value='True',
        description='Launch joint_state_publisher for this car node',
    )
    parameter_overrides_json_la = DeclareLaunchArgument(
        'parameter_overrides_json',
        default_value='{}',
        description='JSON object with additional car node parameter overrides',
    )

    # start nodes and use args to set parameters
    waywiser_car_node_launch_action = OpaqueFunction(function=waywiser_car_node_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(vehicle_config_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(publish_map_to_odom_tf_la)
    ld.add_action(node_name_la)
    ld.add_action(enable_autopilot_component_la)
    ld.add_action(cmd_vel_in_la)
    ld.add_action(cmd_vel_out_la)
    ld.add_action(publish_joint_state_publisher_la)
    ld.add_action(parameter_overrides_json_la)

    # start nodes
    ld.add_action(waywiser_car_node_launch_action)

    return ld


def waywiser_car_node_launch(context):
    nodes = []
    vehicle_config = FileUtils.get_full_file_path(
        LaunchConfiguration('vehicle_config').perform(context)
    )
    if vehicle_config == '':
        print('vehicle_config is empty! Skipping launching waywiser_car_node.')
        return nodes
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    publish_world_to_odom_tf = LaunchConfiguration('publish_world_to_odom_tf').perform(context)
    node_name = LaunchConfiguration('node_name').perform(context)
    cmd_vel_in = LaunchConfiguration('cmd_vel_in').perform(context)
    cmd_vel_out = LaunchConfiguration('cmd_vel_out').perform(context)
    publish_joint_state_publisher = (
        LaunchConfiguration('publish_joint_state_publisher').perform(context).lower()
        in ['true', '1', 'yes']
    )

    use_sim_time_raw = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_raw.lower() in ['true', '1', 'yes']

    node_params_dict = RosUtils.get_node_params(vehicle_config, 'waywiser_car_node')
    enable_autopilot_component = LaunchConfiguration('enable_autopilot_component').perform(
        context
    )
    if enable_autopilot_component != '':
        node_params_dict['enable_autopilot_component'] = (
            enable_autopilot_component.lower() in ['true', '1', 'yes']
        )

    parameter_overrides_json = LaunchConfiguration('parameter_overrides_json').perform(context)
    try:
        parameter_overrides = json.loads(parameter_overrides_json or '{}')
        if not isinstance(parameter_overrides, dict):
            raise ValueError('parameter_overrides_json must decode to a JSON object')
        node_params_dict.update(parameter_overrides)
    except (json.JSONDecodeError, ValueError) as exc:
        print(f'Ignoring invalid parameter_overrides_json: {exc}')

    publish_world_to_odom_tf = publish_world_to_odom_tf.lower() in [
        'true',
        '1',
        'yes',
    ]

    if 'publish_world_to_odom_tf' in node_params_dict:
        node_params_dict['publish_world_to_odom_tf'] = (
            publish_world_to_odom_tf and node_params_dict['publish_world_to_odom_tf']
        )
    else:
        node_params_dict['publish_world_to_odom_tf'] = publish_world_to_odom_tf

    if 'urdf_file' in node_params_dict:
        robot_state_publisher_node = get_robot_state_publisher_node(
            context, node_params_dict, use_sim_time, frame_prefix
        )
        if robot_state_publisher_node is not None:
            nodes.append(robot_state_publisher_node)
        else:
            node_params_dict.pop('urdf_file')

    joint_states_topic = node_params_dict.get('joint_states_topic', '/joint_states')

    nodes.append(
        Node(
            package='waywiser_core',
            executable='waywiser_car_node',
            name=node_name,
            parameters=[
                node_params_dict,
                {
                    'use_sim_time': use_sim_time,
                    'frame_prefix': frame_prefix,
                },
            ],
            remappings=[('/cmd_vel_in', cmd_vel_in), ('/cmd_vel_out', cmd_vel_out)],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen',
            emulate_tty=True,
            # prefix='xterm -e gdb -q -ex run --args',
        )
    )

    if publish_joint_state_publisher:
        joint_state_publisher_name = (
            'ego_veh_joint_state_publisher'
            if node_name == 'waywiser_car_node'
            else f'{node_name}_joint_state_publisher'
        )
        nodes.append(
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name=joint_state_publisher_name,
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'source_list': [joint_states_topic],
                        'frame_prefix': frame_prefix,
                    }
                ],
            )
        )

    return nodes
