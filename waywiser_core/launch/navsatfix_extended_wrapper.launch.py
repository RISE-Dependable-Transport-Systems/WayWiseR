from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    # === Launch arguments ===
    config_la = DeclareLaunchArgument(
        'config',
        default_value='',
        description='Full path to YAML parameter file for navsatfix_extended_wrapper_node',
    )

    node_name_la = DeclareLaunchArgument(
        'node_name',
        default_value='navsatfix_extended_wrapper_node',
        description='Name of the NavSatFix -> NavSatFixExtended conversion node',
    )
    nav_sat_fix_input_topic_la = DeclareLaunchArgument(
        'nav_sat_fix_input_topic',
        default_value='',
        description='Input topic for NavSatFix messages (overrides YAML)',
    )

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/Ignition) clock',
    )

    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='rover/',
        description='Prefix to publish robot transforms in',
    )
    publish_tf_la = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Whether to optionally publish TF from GPS data',
    )

    # === Opaque Function for Node Initialisation ===
    navsatfix_extended_wrapper_node_launch_action = OpaqueFunction(
        function=navsatfix_extended_wrapper_node_launch
    )

    # === Launch Description ===
    ld = LaunchDescription()

    ld.add_action(config_la)
    ld.add_action(node_name_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(publish_tf_la)
    ld.add_action(nav_sat_fix_input_topic_la)

    ld.add_action(navsatfix_extended_wrapper_node_launch_action)

    return ld


def navsatfix_extended_wrapper_node_launch(context):
    config = FileUtils.get_full_file_path(LaunchConfiguration('config').perform(context))
    node_name = LaunchConfiguration('node_name').perform(context)

    node_params_dict = RosUtils.get_node_params(config, node_name)
    nav_sat_fix_input_topic = LaunchConfiguration('nav_sat_fix_input_topic').perform(context)
    if nav_sat_fix_input_topic != '':
        node_params_dict['nav_sat_fix_input_topic'] = nav_sat_fix_input_topic

    # === Node definition ===
    navsatfix_extended_wrapper_node = Node(
        package='waywiser_core',
        executable='navsatfix_extended_wrapper_node',
        name=node_name,
        parameters=[
            node_params_dict,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    return [navsatfix_extended_wrapper_node]
