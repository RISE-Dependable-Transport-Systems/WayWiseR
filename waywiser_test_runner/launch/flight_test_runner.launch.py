import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from waywiser_py.waywiser_utils import RosUtils


def generate_launch_description():
    waywiser_test_runner_dir = get_package_share_directory('waywiser_test_runner')

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    drone_name_la = DeclareLaunchArgument(
        'drone_name', default_value='drone', description='Drone namespace'
    )
    runner_config_la = DeclareLaunchArgument(
        'runner_config',
        default_value=os.path.join(
            waywiser_test_runner_dir, 'config', 'drone_flight_test_runner.yaml'
        ),
        description='Full path to flight test runner params file',
    )
    test_configurations_json_la = DeclareLaunchArgument(
        'test_configurations_json',
        default_value=os.path.join(
            waywiser_test_runner_dir,
            'config',
            'drone_flight_test_configurations.json',
        ),
        description='Full path to flight test configurations JSON file',
    )
    test_config_start_index_la = DeclareLaunchArgument(
        'test_config_start_index',
        default_value='',
        description='Optional 1-based expanded test configuration index to start from',
    )
    test_config_end_index_la = DeclareLaunchArgument(
        'test_config_end_index',
        default_value='',
        description='Optional 1-based expanded test configuration index to end at',
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_la)
    ld.add_action(drone_name_la)
    ld.add_action(runner_config_la)
    ld.add_action(test_configurations_json_la)
    ld.add_action(test_config_start_index_la)
    ld.add_action(test_config_end_index_la)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


def launch_setup(context):
    runner_config = LaunchConfiguration('runner_config').perform(context)
    test_configurations_json = LaunchConfiguration('test_configurations_json').perform(context)
    test_config_start_index = LaunchConfiguration('test_config_start_index').perform(context)
    test_config_end_index = LaunchConfiguration('test_config_end_index').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    runner_params = RosUtils.get_node_params(runner_config, 'flight_test_runner')
    runner_params['test_configurations_json_path'] = test_configurations_json
    if test_config_start_index:
        runner_params['test_config_start_index'] = int(test_config_start_index)
    if test_config_end_index:
        runner_params['test_config_end_index'] = int(test_config_end_index)

    nodes = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('drone_name')),
            Node(
                package='waywiser_test_runner',
                executable='flight_test_runner_node.py',
                name='flight_test_runner',
                output='screen',
                parameters=[runner_params, {'use_sim_time': use_sim_time}],
                sigterm_timeout=['30'],
            ),
        ]
    )

    return [nodes]
