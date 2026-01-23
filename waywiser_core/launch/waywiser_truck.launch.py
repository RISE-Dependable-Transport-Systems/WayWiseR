from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_description_py.waywiser_description_utils import get_robot_state_publisher_node
from waywiser_py.waywiser_utils import FileUtils, RosUtils


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
    publish_map_to_odom_tf_la = DeclareLaunchArgument(
        'publish_world_to_odom_tf', default_value='True', description='Publish map to odom tf'
    )

    # start nodes and use args to set parameters
    waywiser_truck_node_launch_action = OpaqueFunction(function=waywiser_truck_node_launch)

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(vehicle_config_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(publish_map_to_odom_tf_la)

    # start nodes
    ld.add_action(waywiser_truck_node_launch_action)

    return ld


def waywiser_truck_node_launch(context):
    nodes = []
    vehicle_config = FileUtils.get_full_file_path(
        LaunchConfiguration('vehicle_config').perform(context)
    )
    if vehicle_config == '':
        print('vehicle_config is empty! Skipping launching waywiser_truck_node.')
        return nodes
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    publish_world_to_odom_tf = LaunchConfiguration('publish_world_to_odom_tf').perform(context)

    use_sim_time_raw = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_raw.lower() in ['true', '1', 'yes']

    node_params_dict = RosUtils.get_node_params(vehicle_config, 'waywiser_truck_node')
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
            executable='waywiser_truck_node',
            name='waywiser_truck_node',
            parameters=[
                node_params_dict,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frame_prefix': frame_prefix,
                },
            ],
            remappings=[('/cmd_vel_in', 'twist_safety_vel'), ('/cmd_vel_out', 'cmd_vel_out')],
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
                    'source_list': [joint_states_topic],
                    'frame_prefix': frame_prefix,
                }
            ],
        )
    )

    return nodes
