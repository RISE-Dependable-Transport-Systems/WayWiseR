import json
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    world_name_la = DeclareLaunchArgument(
        'world_name',
        default_value='car_world',
        description='Name of the world to spawn models in',
    )
    spawn_config_file_la = DeclareLaunchArgument(
        'spawn_config_file',
        default_value='',
        description='Path to a JSON file containing spawn configurations',
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(world_name_la)
    ld.add_action(spawn_config_file_la)

    # spawn models if spawn_config_file is set
    ld.add_action(OpaqueFunction(function=spawn_models))

    return ld


def spawn_models(context):
    spawn_action = []
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name').perform(context)
    spawn_config_file = LaunchConfiguration('spawn_config_file').perform(context)

    # JSON based spawning
    if spawn_config_file != '':
        json_actions = spawn_from_json(spawn_config_file, world_name, use_sim_time)
        if json_actions:
            spawn_action.extend(json_actions)

    return spawn_action


def spawn_from_json(config_file, world_name, use_sim_time):
    actions = []
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)

        for model in config.get('sdf_models', []):
            path = model.get('path')
            if path:
                actions.extend(
                    create_sdf_spawn_actions(
                        path,
                        world_name,
                        use_sim_time,
                        model.get('bridge_config'),
                        model.get('pose'),
                        model.get('name'),
                    )
                )

        for model in config.get('topic_models', []):
            topic = model.get('topic')
            if topic:
                actions.extend(
                    create_topic_spawn_actions(
                        topic,
                        world_name,
                        use_sim_time,
                        model.get('bridge_config'),
                        model.get('pose'),
                        model.get('name'),
                    )
                )

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


def create_sdf_spawn_actions(
    sdf_path, world_name, use_sim_time, bridge_path=None, pose=None, name=None
):
    actions = []
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
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    )

    if bridge_path:
        actions.append(create_bridge_node(bridge_path, use_sim_time))

    return actions


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
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    )

    if bridge_path:
        actions.append(create_bridge_node(bridge_path, use_sim_time))

    return actions


def create_bridge_node(bridge_path, use_sim_time):
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': bridge_path},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )
