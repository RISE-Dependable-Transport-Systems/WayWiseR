import ast
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_dir = get_package_share_directory('waywiser_gazebo')

    # args that can be set from the command line or a default will be used
    gazebo_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_dir, 'worlds/car_world.sdf'),
        description='Full path to gazebo sdf file',
    )
    gazebo_bridge_la = DeclareLaunchArgument(
        'gazebo_bridge',
        default_value=os.path.join(gazebo_dir, 'config/default_gazebo_bridges.yaml'),
        description='Full path to gazebo bridge file',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    ign_gazebo_resource_paths_la = DeclareLaunchArgument(
        'ign_gazebo_resource_paths',
        default_value='',
        description='Paths to additional model resources as a list',
    )

    # gazebo bridge
    ros_gz_bridge_node = OpaqueFunction(function=create_bridge_node)

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

    map_frame_transform = OpaqueFunction(function=create_map_frame_transform)

    # create launch description
    ld = LaunchDescription()
    ld.add_action(ign_gazebo_resource_paths_la)
    ld.add_action(OpaqueFunction(function=set_ign_resources_path))

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_la)
    ld.add_action(gazebo_bridge_la)

    # run gazebo launch file
    ld.add_action(gazebo)
    ld.add_action(map_frame_transform)

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

    # print('ign_resources_path:{}', ign_resources_path)
    ign_resources_path_set_action = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', ':'.join(ign_resources_path)
    )
    return [ign_resources_path_set_action]


def create_bridge_node(context):
    world_path = Path(LaunchConfiguration('world').perform(context)).resolve()
    world_name = read_world_name(world_path)
    bridge_config = create_runtime_bridge_config(
        LaunchConfiguration('gazebo_bridge').perform(context), world_name
    )

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=['--ros-args', '-p', ['config_file:=', bridge_config]],
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }
            ],
        )
    ]


def create_map_frame_transform(context):
    world_path = Path(LaunchConfiguration('world').perform(context)).resolve()
    world_name = read_world_name(world_path)

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x',
                '0',
                '--y',
                '0',
                '--z',
                '0',
                '--roll',
                '0',
                '--pitch',
                '0',
                '--yaw',
                '0',
                '--frame-id',
                world_name,
                '--child-frame-id',
                'map',
            ],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        )
    ]


def create_runtime_bridge_config(bridge_config_path, world_name):
    import yaml

    with open(bridge_config_path, 'r', encoding='utf-8') as bridge_file:
        config = yaml.safe_load(bridge_file) or []

    if not isinstance(config, list):
        raise RuntimeError(f'Gazebo bridge config must be a list: {bridge_config_path}')

    for entry in config:
        gz_topic_name = entry.get('gz_topic_name')
        if isinstance(gz_topic_name, str):
            entry['gz_topic_name'] = gz_topic_name.format(world_name=world_name)

    temp_config = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='waywiser_gazebo_bridge_',
        suffix='.yaml',
        delete=False,
    )
    with temp_config:
        yaml.safe_dump(config, temp_config)

    return temp_config.name


def read_world_name(world_path: Path):
    if world_path.is_file():
        try:
            tree = ET.parse(world_path)
            world_element = tree.getroot().find('world')
            if world_element is not None and world_element.get('name'):
                return str(world_element.get('name'))
        except ET.ParseError:
            pass
    return world_path.stem
