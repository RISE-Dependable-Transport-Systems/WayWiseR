import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    rviz_la = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='Full path of rviz display config file or path to their directory',
    )
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock',
    )

    # start nodes and use args to set parameters
    launch_setup_action = OpaqueFunction(function=launch_setup)

    # create launch description
    ld = LaunchDescription()

    # declare launch args
    ld.add_action(rviz_la)
    ld.add_action(use_sim_time_la)

    # start nodes
    ld.add_action(launch_setup_action)

    return ld


def launch_setup(context):
    rviz2_dir = get_package_share_directory('waywiser_rviz2')
    default_rviz_config_dir = os.path.join(rviz2_dir, 'rviz')
    default_rviz_config_file = os.path.join(default_rviz_config_dir, 'odom_reference_frame.rviz')

    rviz_config_file_full_path = ''
    rviz_config_file_or_dir = LaunchConfiguration('rviz_config').perform(context)

    if rviz_config_file_or_dir.strip():
        # Check if the path is a file
        if os.path.isfile(rviz_config_file_or_dir):
            rviz_config_file_full_path = os.path.realpath(rviz_config_file_or_dir)
            print(f'Using "{rviz_config_file_full_path}" config file for rviz.')

        # Check if the path is a directory
        elif os.path.isdir(rviz_config_file_or_dir):
            rviz_config_file_full_path = promt_user(rviz_config_file_or_dir)
        else:
            print(
                f'"{rviz_config_file_or_dir}" doesn\'t exist or is neither a file nor a directory. Do you want to proceed with default rviz config files?'  # noqa
            )
            rviz_config_file_full_path = promt_user(default_rviz_config_dir)
    else:
        rviz_config_file_full_path = promt_user(default_rviz_config_dir)

    if not rviz_config_file_full_path.strip():
        rviz_config_file_full_path = default_rviz_config_file
        print(f'Starting rviz with default config "{rviz_config_file_full_path}"')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_full_path],
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    return [rviz_node]


def promt_user(rviz_config_dir):
    # Prompt to choose files from dir
    rviz_config_files = os.listdir(rviz_config_dir)
    rviz_config_files = [file for file in rviz_config_files if file.endswith('.rviz')]
    if len(rviz_config_files) > 0:
        print(f'The directory "{rviz_config_dir}" contains the following rviz config files:')
        for i, file in enumerate(rviz_config_files, start=1):
            print(f'{i}. {file}')
        print(f'{len(rviz_config_files) + 1}. Input a new path')
        # Prompt the user to select a file by its number or input a new path
        selected_index = input('Enter the number corresponding to the file you want: ')

        # Validate the user input
        try:
            selected_index = int(selected_index)
            if 1 <= selected_index <= len(rviz_config_files):
                rviz_config_file_full_path = os.path.realpath(
                    os.path.join(rviz_config_dir, rviz_config_files[selected_index - 1])
                )
                print(f'Using "{rviz_config_file_full_path}" config file for rviz.')
            elif selected_index == len(rviz_config_files) + 1:
                rviz_config_file_full_path = os.path.expanduser(
                    input(
                        'Enter a new full path of an existing rviz config file or a new file name with full path: '  # noqa
                    )
                )
                if rviz_config_file_full_path.strip():
                    _, file_extension = os.path.splitext(rviz_config_file_full_path)
                    if file_extension.lower() != '.rviz':
                        rviz_config_file_full_path = rviz_config_file_full_path + '.rviz'
                    print(f'Using "{rviz_config_file_full_path}" config file for rviz.')
                else:
                    print('Empty input.')
            else:
                print('Invalid number.')
                rviz_config_file_full_path = ''
        except ValueError:
            print('Invalid input.')
            rviz_config_file_full_path = ''
    else:
        print(f'The directory "{rviz_config_dir}" is empty.')
        rviz_config_file_full_path = ''

    return rviz_config_file_full_path
