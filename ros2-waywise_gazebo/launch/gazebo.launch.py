import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    description_dir = get_package_share_directory('ros2-waywise_description')
    gazebo_dir = get_package_share_directory('ros2-waywise_gazebo')

    # args that can be set from the command line
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_prefix = LaunchConfiguration('frame_prefix')

    # args that can be set from the command line or a default will be used
    robot_state_publisher_la = DeclareLaunchArgument(
        'model', default_value=os.path.join(description_dir, 'urdf/robot.urdf.xacro'),
        description='Full path to robot urdf file')
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation/Gazebo clock')
    frame_prefix_la = DeclareLaunchArgument(
        'frame_prefix', default_value='/',
        description='Prefix to publish robot transforms in')

    # start nodes and use args to set parameters
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', model, ' sim_mode:=', 'True']),
            'use_sim_time': use_sim_time,
            'frame_prefix': frame_prefix,
        }]
    )

    # include launch files
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': '-r ' +  os.path.join(gazebo_dir, 'worlds/car_world.sdf')}.items()
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description'],
        output='screen')

    # gazebo bridge
    bridge = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'),
            'run ros_gz_bridge parameter_bridge  --ros-args -p config_file:=' + os.path.join(gazebo_dir, 'config/ros_gazebo_bridges.yaml')
            ],
        output='screen',
        shell=True
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(robot_state_publisher_la)
    ld.add_action(use_sim_time_la)
    ld.add_action(frame_prefix_la)

    # start robot_state_publisher_node
    ld.add_action(robot_state_publisher_node)

    # run gazebo launch file
    ld.add_action(gazebo)

    # spawn robot in gazebo
    ld.add_action(spawn_entity)

    # setup gazebo bridge
    ld.add_action(bridge)

    return ld
