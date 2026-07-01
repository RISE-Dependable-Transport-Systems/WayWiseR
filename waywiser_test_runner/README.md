## Examples

    ros2 launch waywiser_test_runner track_test_runner.launch.py
    ros2 launch waywiser_test_runner flight_test_runner.launch.py use_sim_time:=True

## Topic Perturbation

`topic_perturbation_node.py` is a generic topic relay for validation runs. It can pass
messages through, delay them, drop them, replay the last received message, and apply
Twist-specific numeric transforms. It currently lives in `waywiser_test_runner`
because the test runners launch and control it; if it becomes a general runtime tool,
move it to a dedicated package such as `waywiser_topic_tools` rather than the
top-level `waywiser` package.
