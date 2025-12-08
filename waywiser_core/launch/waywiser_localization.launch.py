from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from waywiser_description_py.waywiser_description_utils import get_scaled_urdf_string
import yaml

from waywiser_py.waywiser_utils import get_full_file_path


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    localization_config_la = DeclareLaunchArgument(
        'localization_config',
        default_value='',
        description='Full path to params file of waywiser_localization',
    )
    localization_node_name_la = DeclareLaunchArgument(
        'localization_node_name',
        default_value='waywiser_localization_node',
        description='Name of the node to be launched',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )
    # start nodes and use args to set parameters
    waywiser_localization_node_launch_action = OpaqueFunction(
        function=waywiser_localization_node_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch arg
    ld.add_action(localization_config_la)
    ld.add_action(localization_node_name_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(waywiser_localization_node_launch_action)

    return ld


def waywiser_localization_node_launch(context):
    nodes = []
    localization_config = get_full_file_path(
        LaunchConfiguration('localization_config').perform(context)
    )
    if localization_config == '':
        print('localization_config is empty! Skipping launching waywiser_localization_node.')
        return nodes

    localization_node_name = LaunchConfiguration('localization_node_name').perform(context)
    use_sim_time_raw = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_raw.lower() in ['true', '1', 'yes']
    with open(localization_config, 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f)
        node_params_dict = config_data['/**']['ros__parameters']
        node_params_dict.update(config_data[localization_node_name]['ros__parameters'])

        if 'urdf_file' in node_params_dict:
            urdf_scale = 1.0
            if 'urdf_scale' in node_params_dict:
                urdf_scale = node_params_dict['urdf_scale']

            urdf_extra_args = ''
            if 'urdf_extra_args' in node_params_dict:
                urdf_extra_args = node_params_dict['urdf_extra_args']

            urdf_file_string = get_scaled_urdf_string(
                node_params_dict['urdf_file'], urdf_scale, urdf_extra_args
            )
            if urdf_file_string is not None:
                node_params_dict['urdf_file'] = urdf_file_string
            else:
                node_params_dict.pop('urdf_file')

        nodes.append(
            Node(
                package='waywiser_core',
                executable='waywiser_localization_node',
                name=localization_node_name,
                parameters=[
                    node_params_dict,
                    {'use_sim_time': use_sim_time},
                ],
                output='screen',
                emulate_tty=True,
                # prefix='xterm -e gdb -q -ex run --args',
            )
        )

    return nodes
