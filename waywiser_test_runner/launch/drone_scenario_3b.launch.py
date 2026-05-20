import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from waywiser_py.waywiser_utils import RosUtils


def generate_launch_description():
    waywiser_test_runner_dir = get_package_share_directory('waywiser_test_runner')
    waywiser_gazebo_dir = get_package_share_directory('waywiser_gazebo')
    waywiser_teleop_dir = get_package_share_directory('waywiser_teleop')
    waywiser_rviz2_dir = get_package_share_directory('waywiser_rviz2')

    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation/Gazebo clock'
    )
    world_la = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(waywiser_gazebo_dir, 'worlds/bounded_world.sdf'),
        description='Full path to gazebo sdf file',
    )
    rviz_config_la = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(waywiser_rviz2_dir, 'config/map_reference_frame_drone.rviz'),
        description='Full path of rviz display config file or path to their directory',
    )
    teleop_config_la = DeclareLaunchArgument(
        'teleop_config',
        default_value=os.path.join(waywiser_teleop_dir, 'config/teleop.yaml'),
        description='Full path to teleop params file',
    )
    teleop_la = DeclareLaunchArgument(
        'teleop', default_value='True', description='Launch teleop'
    )
    rviz2_la = DeclareLaunchArgument('rviz2', default_value='False', description='Launch rviz2')
    drone_name_la = DeclareLaunchArgument(
        'drone_name', default_value='drone', description='Drone namespace'
    )
    drone_config_la = DeclareLaunchArgument(
        'drone_config',
        default_value=os.path.join(waywiser_gazebo_dir, 'config', 'drone_scenario_3b.yaml'),
        description='Full path to Scenario 3b drone config overlay',
    )
    scenario_runner_config_la = DeclareLaunchArgument(
        'scenario_runner_config',
        default_value=os.path.join(
            waywiser_test_runner_dir, 'config', 'drone_scenario_3b_runner.yaml'
        ),
        description='Full path to params file for the Scenario 3b runner and emulator',
    )
    scenario_configurations_json_la = DeclareLaunchArgument(
        'scenario_configurations_json',
        default_value=os.path.join(
            waywiser_test_runner_dir, 'config', 'drone_scenario_3b_test_configurations.json'
        ),
        description='Full path to Scenario 3b JSON configuration file',
    )

    launch_setup_action = OpaqueFunction(function=launch_setup)

    ld = LaunchDescription()
    ld.add_action(use_sim_time_la)
    ld.add_action(world_la)
    ld.add_action(rviz_config_la)
    ld.add_action(teleop_config_la)
    ld.add_action(teleop_la)
    ld.add_action(rviz2_la)
    ld.add_action(drone_name_la)
    ld.add_action(drone_config_la)
    ld.add_action(scenario_runner_config_la)
    ld.add_action(scenario_configurations_json_la)
    ld.add_action(launch_setup_action)
    return ld


def launch_setup(context):
    waywiser_dir = get_package_share_directory('waywiser')

    scenario_runner_config = LaunchConfiguration('scenario_runner_config').perform(context)
    scenario_configurations_json = LaunchConfiguration('scenario_configurations_json').perform(
        context
    )
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    drone_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(waywiser_dir, 'launch', 'gazebo_drone_px4_zenoh.launch.py')]
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'teleop_config': LaunchConfiguration('teleop_config'),
            'teleop': LaunchConfiguration('teleop'),
            'rviz2': LaunchConfiguration('rviz2'),
            'drone_config': LaunchConfiguration('drone_config'),
            'drone_name': LaunchConfiguration('drone_name'),
        }.items(),
    )

    runner_params = RosUtils.get_node_params(scenario_runner_config, 'drone_scenario_runner')
    runner_params['scenario_configurations_json_path'] = scenario_configurations_json
    emulator_params = RosUtils.get_node_params(
        scenario_runner_config, 'communication_channel_emulator'
    )

    scenario_nodes = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('drone_name')),
            Node(
                package='waywiser_test_runner',
                executable='communication_channel_emulator.py',
                name='communication_channel_emulator',
                output='screen',
                parameters=[emulator_params, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='waywiser_test_runner',
                executable='drone_scenario_runner.py',
                name='drone_scenario_runner',
                output='screen',
                parameters=[runner_params, {'use_sim_time': use_sim_time}],
            ),
        ]
    )

    return [drone_stack, scenario_nodes]