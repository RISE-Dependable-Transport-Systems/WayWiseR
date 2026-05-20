## Examples

    ros2 launch waywiser_test_runner track_test_runner.launch.py
    ros2 launch waywiser_test_runner track_test_runner.launch.py use_sim_time:=True

## Scenario 3b

    ros2 launch waywiser_test_runner drone_scenario_3b.launch.py

This launch composes the Zenoh PX4 drone stack with:
- a namespaced `communication_channel_emulator` that rewrites `teleop_mux_vel`
- a namespaced `drone_scenario_runner` that arms the vehicle, requests auto lift-off, publishes nominal `teleop_mux_vel` commands, enables the attack window, and records rosbag data

By default the Scenario 3b overlay repoints `onboard_twist_mux` to `teleop_mux_vel_compromised`, preserving the downstream safety chain through `onboard_mux_vel`, `twist_safety_vel`, and `cmd_vel_out`.
