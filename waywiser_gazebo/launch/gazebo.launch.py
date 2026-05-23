import ast
from os import environ
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros2pkg.api import get_package_names


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
    use_nvidia_gpu_la = DeclareLaunchArgument(
        'use_nvidia_gpu',
        default_value='True',
        description='Use NVIDIA PRIME offload environment variables for Gazebo rendering',
    )
    gazebo_sim_version_la = DeclareLaunchArgument(
        'gazebo_sim_version',
        default_value='6',
        description='Gazebo Sim major version to launch.',
    )
    launch_bridge_la = DeclareLaunchArgument(
        'launch_bridge',
        default_value='True',
        description='Launch default Gazebo bridge',
    )
    launch_map_frame_transform_la = DeclareLaunchArgument(
        'launch_map_frame_transform',
        default_value='True',
        description='Launch static map frame transform',
    )

    # nvidia GPU offload env vars setup
    use_nvidia_gpu = IfCondition(LaunchConfiguration('use_nvidia_gpu'))
    nvidia_gpu_env = GroupAction(
        actions=[
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
            SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
            SetEnvironmentVariable(
                '__EGL_VENDOR_LIBRARY_FILENAMES',
                '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
            ),
        ],
        condition=use_nvidia_gpu,
    )

    # gazebo bridge
    ros_gz_bridge_node = GroupAction(
        actions=[OpaqueFunction(function=create_bridge_node)],
        condition=IfCondition(LaunchConfiguration('launch_bridge')),
    )

    gazebo = OpaqueFunction(function=create_gazebo_process)

    map_frame_transform = GroupAction(
        actions=[OpaqueFunction(function=create_map_frame_transform)],
        condition=IfCondition(LaunchConfiguration('launch_map_frame_transform')),
    )

    # create launch description
    ld = LaunchDescription()
    ld.add_action(ign_gazebo_resource_paths_la)
    ld.add_action(OpaqueFunction(function=set_ign_resources_path))

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(gazebo_la)
    ld.add_action(gazebo_bridge_la)
    ld.add_action(use_nvidia_gpu_la)
    ld.add_action(gazebo_sim_version_la)
    ld.add_action(launch_bridge_la)
    ld.add_action(launch_map_frame_transform_la)

    # run Nvidia GPU setup action
    ld.add_action(nvidia_gpu_env)

    # run gazebo launch file
    ld.add_action(gazebo)
    ld.add_action(map_frame_transform)

    # setup gazebo bridge
    ld.add_action(ros_gz_bridge_node)

    return ld


def create_gazebo_process(context):
    world_path = LaunchConfiguration('world').perform(context)
    gazebo_sim_version = LaunchConfiguration('gazebo_sim_version').perform(context)
    gazebo_env = gazebo_ros_paths_environment(context)
    ign_executable = shutil.which('ign')
    gz_executable = shutil.which('gz')

    if gazebo_sim_version != '6':
        if not gz_executable:
            raise RuntimeError(
                f'Unable to find gz executable for Gazebo Sim {gazebo_sim_version}'
            )
        cmd = [
            'ruby',
            gz_executable,
            'sim',
            '-r',
            world_path,
            '--force-version',
            gazebo_sim_version,
        ]
    elif ign_executable:
        cmd = ['ruby', ign_executable, 'gazebo', '-r', world_path, '--force-version', '6']
    elif gz_executable:
        cmd = ['ruby', gz_executable, 'sim', '-r', world_path, '--force-version', '6']
    else:
        raise RuntimeError('Unable to find ign or gz executable for Gazebo Sim')

    return [
        ExecuteProcess(
            cmd=cmd,
            output={'stdout': 'log', 'stderr': 'log'},
            additional_env=gazebo_env,
        )
    ]


def gazebo_ros_paths_environment(context):
    gazebo_model_path = []
    gazebo_plugin_path = []
    gazebo_media_path = []

    for package_name in get_package_names():
        package_share_path = get_package_share_directory(package_name)
        package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
        if not os.path.isfile(package_file_path):
            continue
        try:
            package = parse_package(package_file_path)
        except InvalidPackage:
            continue
        for export in package.exports:
            if export.tagname != 'gazebo_ros':
                continue
            if 'gazebo_model_path' in export.attributes:
                gazebo_model_path.append(
                    export.attributes['gazebo_model_path'].replace(
                        '${prefix}', package_share_path
                    )
                )
            if 'plugin_path' in export.attributes:
                gazebo_plugin_path.append(
                    export.attributes['plugin_path'].replace('${prefix}', package_share_path)
                )
            if 'gazebo_media_path' in export.attributes:
                gazebo_media_path.append(
                    export.attributes['gazebo_media_path'].replace(
                        '${prefix}', package_share_path
                    )
                )

    resource_paths = [
        os.path.join(get_package_share_directory('waywiser_description'), 'sdf'),
        str(Path(get_package_share_directory('waywiser_description')).parent.absolute()),
    ]
    input_resource_paths = LaunchConfiguration('ign_gazebo_resource_paths').perform(context)
    if input_resource_paths != '':
        input_resource_paths = ast.literal_eval(input_resource_paths)
        if isinstance(input_resource_paths, list):
            resource_paths.extend(input_resource_paths)

    model_paths = os.pathsep.join(gazebo_model_path + gazebo_media_path + resource_paths)
    plugin_paths = os.pathsep.join(gazebo_plugin_path)
    return {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(
            [
                environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                environ.get('LD_LIBRARY_PATH', default=''),
                plugin_paths,
            ]
        ),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.pathsep.join(
            [
                environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                environ.get('LD_LIBRARY_PATH', default=''),
                plugin_paths,
            ]
        ),
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join(
            [
                environ.get('GZ_SIM_RESOURCE_PATH', default=''),
                model_paths,
            ]
        ),
        'IGN_GAZEBO_RESOURCE_PATH': os.pathsep.join(
            [
                environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''),
                model_paths,
            ]
        ),
    }


def set_ign_resources_path(context):
    # Fetch existing resource path environment variables
    existing_paths = []
    for var_name in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH'):
        val = os.environ.get(var_name, '')
        if val:
            existing_paths.extend(val.split(':'))

    ign_resources_path = set(existing_paths)
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

    joined_paths = ':'.join(ign_resources_path)
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', joined_paths),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', joined_paths),
    ]


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
            output='log',
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
