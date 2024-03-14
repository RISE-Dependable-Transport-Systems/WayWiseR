import ast
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
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
    model_sdf_paths_la = DeclareLaunchArgument(
        'model_sdf_paths',
        default_value='',
        description='Paths to model sdf files as a list',
    )

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(world_name_la)
    ld.add_action(model_sdf_paths_la)

    # spawn models if model_sdf_paths is set
    ld.add_action(OpaqueFunction(function=spawn_models))

    return ld


def spawn_models(context):
    spawn_action = []
    model_sdf_paths = LaunchConfiguration('model_sdf_paths').perform(context)
    if model_sdf_paths != '':
        model_sdf_paths = ast.literal_eval(model_sdf_paths)
        if isinstance(model_sdf_paths, list):
            for model_sdf_path in model_sdf_paths:
                # Extracting spawn pose
                spawn_pose = ['0', '0', '0', '0', '0', '0']
                tree = ET.parse(model_sdf_path)
                root = tree.getroot()
                actor_tag = root.find('actor')
                if actor_tag:
                    spawn_pose = actor_tag.find('pose').text.split(' ')
                else:
                    model_tag = root.find('model')
                    if model_tag:
                        spawn_pose = model_tag.find('pose').text.split(' ')

                spawn_action.append(
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        arguments=[
                            '-file',
                            model_sdf_path,
                            '-world',
                            LaunchConfiguration('world_name').perform(context),
                            '-x',
                            spawn_pose[0],  # Assuming x, y, z, roll, pitch, yaw in pose_values
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
                        ],
                        parameters=[
                            {
                                'use_sim_time': LaunchConfiguration('use_sim_time'),
                            }
                        ],
                        output='screen',
                    )
                )

    return spawn_action
