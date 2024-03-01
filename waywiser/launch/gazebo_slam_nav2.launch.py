import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')
    waywiser_twist_safety_dir = get_package_share_directory('waywiser_twist_safety')
    waywiser_slam_dir = get_package_share_directory('waywiser_slam')
    waywiser_nav2_dir = get_package_share_directory('waywiser_nav2')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    gazebo_world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/car_world.sdf'),
        description='Full path to gazebo sdf file',
    )
    enable_collision_monitor_la = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='True',
        description='Use Nav2 collision monitoring',
    )

    # include launch files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_gazebo_dir,
                    'launch',
                    'gazebo.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    twist_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_twist_safety_dir,
                    'launch',
                    'twist_safety.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_collision_monitor': LaunchConfiguration('enable_collision_monitor'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_slam_dir,
                    'launch',
                    'slam.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    waywiser_nav2_dir,
                    'launch',
                    'nav2.launch.py',
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_world_la)
    ld.add_action(enable_collision_monitor_la)

    # start nodes
    ld.add_action(gazebo)
    ld.add_action(twist_safety)
    ld.add_action(slam)
    ld.add_action(nav2)

    return ld
