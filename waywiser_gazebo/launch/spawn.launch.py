import json
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_dir = get_package_share_directory('waywiser_gazebo')

    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_dir, 'worlds/car_world.sdf'),
        description='Full path to Gazebo world SDF file used to detect the Gazebo world.',
    )
    spawn_config_file_la = DeclareLaunchArgument(
        'spawn_config_file',
        default_value='',
        description='Path to a JSON file containing spawn configurations',
    )
    spawn_start_delay_la = DeclareLaunchArgument(
        'spawn_start_delay',
        default_value='0.0',
        description='Delay before spawning the first configured model',
    )
    spawn_interval_la = DeclareLaunchArgument(
        'spawn_interval',
        default_value='1.0',
        description='Delay between model spawn requests from the same config file',
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(world_la)
    ld.add_action(spawn_config_file_la)
    ld.add_action(spawn_start_delay_la)
    ld.add_action(spawn_interval_la)

    # spawn models if spawn_config_file is set
    ld.add_action(OpaqueFunction(function=spawn_models))

    return ld


def spawn_models(context):
    spawn_action = []
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = resolve_world_name(context)
    spawn_config_file = LaunchConfiguration('spawn_config_file').perform(context)
    spawn_start_delay = float(LaunchConfiguration('spawn_start_delay').perform(context))
    spawn_interval = float(LaunchConfiguration('spawn_interval').perform(context))

    # JSON based spawning
    if spawn_config_file != '':
        json_actions = spawn_from_json(
            spawn_config_file, world_name, use_sim_time, spawn_start_delay, spawn_interval
        )
        if json_actions:
            spawn_action.extend(json_actions)

    return spawn_action


def resolve_world_name(context):
    world = LaunchConfiguration('world').perform(context)
    return read_world_name(Path(world).resolve())


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


def spawn_from_json(config_file, world_name, use_sim_time, spawn_start_delay=0.0, spawn_interval=1.0):
    actions = []
    model_index = 0
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)

        for model in config.get('sdf_models', []):
            path = model.get('path')
            if path:
                actions.append(
                    make_timed_model_actions(
                        create_sdf_spawn_actions(
                            path,
                            world_name,
                            use_sim_time,
                            model.get('bridge_config'),
                            model.get('pose'),
                            model.get('name'),
                            model.get('static'),
                        ),
                        spawn_start_delay,
                        spawn_interval,
                        model_index,
                    )
                )
                model_index += 1

        for model in config.get('topic_models', []):
            topic = model.get('topic')
            if topic:
                actions.append(
                    make_timed_model_actions(
                        create_topic_spawn_actions(
                            topic,
                            world_name,
                            use_sim_time,
                            model.get('bridge_config'),
                            model.get('pose'),
                            model.get('name'),
                        ),
                        spawn_start_delay,
                        spawn_interval,
                        model_index,
                    )
                )
                model_index += 1

        for tf in config.get('static_transforms', []):
            frame_id = tf.get('frame_id')
            child_frame_id = tf.get('child_frame_id')
            pose = tf.get('pose', [0, 0, 0, 0, 0, 0])

            if frame_id and child_frame_id:
                pose_vals = []
                if isinstance(pose, str):
                    pose_vals = pose.split(' ')
                elif isinstance(pose, list):
                    pose_vals = [str(v) for v in pose]

                # Default to 0 if missing
                pose_vals += ['0'] * (6 - len(pose_vals))

                actions.append(
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        arguments=[
                            '--x',
                            pose_vals[0],
                            '--y',
                            pose_vals[1],
                            '--z',
                            pose_vals[2],
                            '--roll',
                            pose_vals[3],
                            '--pitch',
                            pose_vals[4],
                            '--yaw',
                            pose_vals[5],
                            '--frame-id',
                            frame_id,
                            '--child-frame-id',
                            child_frame_id,
                        ],
                        parameters=[{'use_sim_time': use_sim_time}],
                        output='screen',
                    )
                )

    except (json.JSONDecodeError, FileNotFoundError):
        return None

    return actions if actions else None


def make_timed_model_actions(actions, spawn_start_delay, spawn_interval, model_index):
    return TimerAction(
        period=spawn_start_delay + (spawn_interval * model_index),
        actions=actions,
    )


def create_sdf_spawn_actions(
    sdf_path, world_name, use_sim_time, bridge_path=None, pose=None, name=None, static=None
):
    actions = []
    sdf_path = create_sdf_with_static_override(sdf_path, static)

    # Extracting spawn pose
    spawn_pose = ['0', '0', '0', '0', '0', '0']

    if pose:
        if isinstance(pose, str):
            pose_vals = pose.split(' ')
        elif isinstance(pose, list):
            pose_vals = [str(v) for v in pose]
        else:
            pose_vals = []

        # Override only provided values, fallback to 0 for missing ones in the override list
        for i in range(min(len(pose_vals), 6)):
            spawn_pose[i] = pose_vals[i]
    else:
        try:
            tree = ET.parse(sdf_path)
            root = tree.getroot()
            actor_tag = root.find('actor')
            if actor_tag:
                spawn_pose = actor_tag.find('pose').text.split(' ')
            else:
                model_tag = root.find('model')
                if model_tag:
                    spawn_pose = model_tag.find('pose').text.split(' ')
        except (ET.ParseError, FileNotFoundError, AttributeError):
            pass

    arguments = [
        '-file',
        sdf_path,
        '-world',
        world_name,
        '-x',
        spawn_pose[0],
        '-y',
        spawn_pose[1],
        '-z',
        spawn_pose[2],
        '-R',
        spawn_pose[3],
        '-P',
        spawn_pose[4],
        '-Y',
        spawn_pose[5],
    ]

    if name:
        arguments.extend(['-name', name])

    actions.append(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=arguments,
            ros_arguments=['--log-level', 'fatal'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    )

    if bridge_path:
        actions.append(create_bridge_node(bridge_path, use_sim_time, world_name))

    return actions


def create_sdf_with_static_override(sdf_path, static):
    static = normalize_optional_bool(static)
    if static is None:
        return sdf_path

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model_tag = root.find('model')
    if model_tag is None:
        raise RuntimeError(f'Cannot set static on SDF without a <model> tag: {sdf_path}')

    static_tag = model_tag.find('static')
    if static_tag is None:
        static_tag = ET.Element('static')
        model_tag.insert(0, static_tag)
    static_tag.text = static

    temp_sdf = tempfile.NamedTemporaryFile(
        mode='wb',
        prefix='waywiser_spawn_',
        suffix='.sdf',
        delete=False,
    )
    with temp_sdf:
        tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)

    return temp_sdf.name


def normalize_optional_bool(value):
    if value is None:
        return None
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, str):
        value = value.strip().lower()
        if value in ['true', '1', 'yes', 'on']:
            return 'true'
        if value in ['false', '0', 'no', 'off']:
            return 'false'
    raise ValueError(f'static must be a boolean value, got: {value}')


def create_topic_spawn_actions(
    topic_name, world_name, use_sim_time, bridge_path=None, pose=None, name=None
):
    actions = []

    arguments = ['-topic', topic_name, '-world', world_name]

    if name:
        arguments.extend(['-name', name])

    if pose:
        pose_vals = []
        if isinstance(pose, str):
            pose_vals = pose.split(' ')
        elif isinstance(pose, list):
            pose_vals = [str(v) for v in pose]

        if len(pose_vals) >= 3:
            arguments.extend(['-x', pose_vals[0], '-y', pose_vals[1], '-z', pose_vals[2]])
        if len(pose_vals) >= 6:
            arguments.extend(['-R', pose_vals[3], '-P', pose_vals[4], '-Y', pose_vals[5]])

    actions.append(
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=arguments,
            ros_arguments=['--log-level', 'fatal'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    )

    if bridge_path:
        actions.append(create_bridge_node(bridge_path, use_sim_time, world_name))

    return actions


def create_bridge_node(bridge_path, use_sim_time, world_name):
    bridge_path = create_runtime_bridge_config(bridge_path, world_name)

    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_path},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )


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
        prefix='waywiser_spawn_bridge_',
        suffix='.yaml',
        delete=False,
    )
    with temp_config:
        yaml.safe_dump(config, temp_config)

    return temp_config.name
