import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waywiser_py.waywiser_utils import FileUtils, RosUtils


def generate_launch_description():
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')

    # args that can be set from the command line or a default will be used
    slam_config_la = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(waywiser_slam_dir, 'config/slam.yaml'),
        description='Full path to params file for slam toolbox',
    )
    node_name_la = DeclareLaunchArgument(
        'node_name',
        default_value='slam_toolbox',
        description='Name of the slam toolbox node',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix', default_value='/', description='Prefix for robot frames'
    )

    # === Opaque Function for Node Initialisation ===
    online_async_slam_toolbox_node_launch_action = OpaqueFunction(
        function=online_async_slam_toolbox_node_launch
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(slam_config_la)
    ld.add_action(node_name_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(frame_prefix_la)

    # start nodes
    ld.add_action(online_async_slam_toolbox_node_launch_action)

    return ld


def online_async_slam_toolbox_node_launch(context):
    config = FileUtils.get_full_file_path(LaunchConfiguration('slam_config').perform(context))
    node_name = LaunchConfiguration('node_name').perform(context)
    node_params_dict = RosUtils.get_node_params(config, node_name)

    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    if frame_prefix != '/' and frame_prefix != '':
        node_params_dict['odom_frame'] = frame_prefix + node_params_dict['odom_frame']
        node_params_dict['base_frame'] = frame_prefix + node_params_dict['base_frame']

    # === Node definition ===
    online_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name=node_name,
        parameters=[
            node_params_dict,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    return [online_async_slam_toolbox_node]
