import ast
import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('waywiser_description')
    gazebo_dir = get_package_share_directory('waywiser_gazebo')

    # args that can be set from the command line or a default will be used
    robot_state_publisher_la = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_dir, 'urdf/robot.urdf.xacro'),
        description='Full path to robot urdf file',
    )
    gazebo_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_dir, 'worlds/car_world.sdf'),
        description='Full path to gazebo sdf file',
    )
    gazebo_bridge_la = DeclareLaunchArgument(
        'gazebo_bridge',
        default_value=os.path.join(gazebo_dir, 'config/ros_gazebo_bridges.yaml'),
        description='Full path to gazebo bridge file',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix',
        default_value='/',
        description='Prefix to publish robot transforms in',
    )
    ign_gazebo_resource_paths_la = DeclareLaunchArgument(
        'ign_gazebo_resource_paths',
        default_value='',
        description='Paths to additional model resources as a list',
    )

    # start nodes and use args to set parameters
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('model'), ' sim_mode:=', 'True']),
                    value_type=str,
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
            }
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen',
    )

    # gazebo bridge
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['--ros-args', '-p', ['config_file:=', LaunchConfiguration('gazebo_bridge')]],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    # include launch files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                )
            ]
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items(),
    )

    # create launch description
    ld = LaunchDescription()
    ld.add_action(ign_gazebo_resource_paths_la)
    ld.add_action(OpaqueFunction(function=set_ign_resources_path))

    # declare launch args
    ld.add_action(robot_state_publisher_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(frame_prefix_la)
    ld.add_action(gazebo_la)
    ld.add_action(gazebo_bridge_la)

    # start robot_state_publisher_node
    ld.add_action(robot_state_publisher_node)

    # run gazebo launch file
    ld.add_action(gazebo)

    # spawn robot in gazebo
    ld.add_action(spawn_robot)

    # setup gazebo bridge
    ld.add_action(ros_gz_bridge_node)

    return ld


def set_ign_resources_path(context):
    # Fetch IGN_GAZEBO_RESOURCE_PATH environment variable
    ign_resources_path = set(str(os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')).split(':'))
    ign_resources_path = {x for x in ign_resources_path if x}

    # Default waywiser-related list of paths to Ignition Gazebo resources
    default_ign_resources_path = [
        os.path.join(get_package_share_directory('waywiser_description'), 'sdf'),
        str(Path(get_package_share_directory('waywiser_description')).parent.absolute()),
    ]
    ign_resources_path.update(set(default_ign_resources_path))

    input_ign_gazebo_resource_paths = LaunchConfiguration('ign_gazebo_resource_paths').perform(
        context
    )
    if input_ign_gazebo_resource_paths != '':
        input_ign_gazebo_resource_paths = ast.literal_eval(input_ign_gazebo_resource_paths)
        if isinstance(input_ign_gazebo_resource_paths, list):
            ign_resources_path.update(set(input_ign_gazebo_resource_paths))

    print('ign_resources_path:{}', ign_resources_path)
    ign_resources_path_set_action = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', ':'.join(ign_resources_path)
    )
    return [ign_resources_path_set_action]
