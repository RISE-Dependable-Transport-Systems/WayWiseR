import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # args that can be set from the command line or a default will be used
    slam_config_la = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(waywiser_slam_dir, 'config/slam.yaml'),
        description='Full path to params file for slam toolbox',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation/Gazebo clock'
    )

    # start nodes and use args to set parameters

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    slam_toolbox_dir,
                    'launch',
                    'online_async_launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_config'),
        }.items(),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(slam_config_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(slam_toolbox)

    return ld
