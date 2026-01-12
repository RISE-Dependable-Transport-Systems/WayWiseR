import os

from ament_index_python import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def get_scaled_urdf_string(urdf_file, urdf_scale=1.0, extra_args='', frame_prefix=''):
    urdf_scaler_script_path = os.path.join(
        get_package_share_directory('waywiser_description'),
        'scripts',
        'scale_urdf.sh',
    )

    urdf_file = os.path.expanduser(urdf_file)
    # If the path is relative, prepend the package's config directory
    if urdf_file:
        if not urdf_file.startswith('/'):
            urdf_file = os.path.join(
                get_package_share_directory('waywiser_description'),
                'urdf',
                urdf_file,
            )

        cmd_args = [
            urdf_scaler_script_path,
            urdf_file,
            str(urdf_scale),
            f"'{frame_prefix}'",
        ]
        if extra_args:
            cmd_args.append(extra_args)

        return ParameterValue(
            Command(' '.join(cmd_args)),
            value_type=str,
        )

    return None


def get_robot_state_publisher_node(node_params_dict, use_sim_time=False, frame_prefix=''):
    node = None
    urdf_scale = 1.0
    if 'urdf_scale' in node_params_dict:
        urdf_scale = node_params_dict['urdf_scale']

    urdf_extra_args = ''
    if 'urdf_extra_args' in node_params_dict:
        urdf_extra_args = node_params_dict['urdf_extra_args']

    urdf_file_string = get_scaled_urdf_string(
        node_params_dict['urdf_file'], urdf_scale, urdf_extra_args, frame_prefix
    )
    if urdf_file_string is not None:
        node_params_dict['urdf_file'] = urdf_file_string
        node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='ego_veh_state_publisher',
            parameters=[
                {
                    'robot_description': urdf_file_string,
                    'use_sim_time': use_sim_time,
                }
            ],
        )
    return node
