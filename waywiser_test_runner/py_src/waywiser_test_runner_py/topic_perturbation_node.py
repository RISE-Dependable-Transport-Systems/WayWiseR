#!/usr/bin/env python3

from copy import deepcopy
import json
import time
from typing import Any

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String

from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS


class TopicPerturbationNode(Node):
    """
    Apply controlled perturbations to a generic ROS topic stream.

    Supported generic modes are passthrough, delay, dropout, and replay. For
    geometry_msgs/msg/Twist, numeric scale/bias/override transforms are also
    available for command-channel experiments.
    """

    def __init__(self):
        super().__init__('topic_perturbation_node')

        self.declare_parameter('message_type', 'geometry_msgs/msg/Twist')
        self.declare_parameter('input_topic', 'teleop_mux_vel')
        self.declare_parameter('output_topic', 'teleop_mux_vel_compromised')
        self.declare_parameter('perturbation_control_topic', '')
        self.declare_parameter('perturbation_profile_topic', '')
        self.declare_parameter('default_profile_json', '{"mode": "passthrough"}')
        self.declare_parameter('start_active', False)
        self.declare_parameter('default_publish_rate_hz', 0.0)
        self.declare_parameter('delay_poll_rate_hz', 50.0)
        # reset topic for clearing state between iterations
        self.declare_parameter('reset_topic', '')

        self.message_type_name = (
            self.get_parameter('message_type').get_parameter_value().string_value
        )
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.control_topic = (
            self.get_parameter('perturbation_control_topic').get_parameter_value().string_value
        )
        self.profile_topic = (
            self.get_parameter('perturbation_profile_topic').get_parameter_value().string_value
        )
        self.default_publish_rate_hz = (
            self.get_parameter('default_publish_rate_hz').get_parameter_value().double_value
        )
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        delay_poll_rate_hz = max(
            1.0, self.get_parameter('delay_poll_rate_hz').get_parameter_value().double_value
        )

        self.active = self.get_parameter('start_active').get_parameter_value().bool_value
        self.active_started_at_s = self._now_s() if self.active else None
        self.current_profile: dict[str, Any] = {'mode': 'passthrough'}
        self.last_input_msg = None
        self.delay_queue = []
        self.replay_history = []
        self.replay_sequence = []
        self.replay_index = 0
        self.replay_empty_warned = False
        self.dropout_passed_cycles = set()
        self.republish_timer = None
        self.current_publish_rate_hz = 0.0

        try:
            self.message_type = get_message(self.message_type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            raise RuntimeError(
                f"Could not load ROS message type '{self.message_type_name}'"
            ) from exc

        default_profile_json = (
            self.get_parameter('default_profile_json').get_parameter_value().string_value
        )
        self._update_profile(default_profile_json, log_invalid=False)

        self.output_publisher = self.create_publisher(self.message_type, self.output_topic, 10)
        self.create_subscription(self.message_type, self.input_topic, self.input_callback, 10)
        self.create_subscription(
            Bool,
            self.control_topic,
            self.active_control_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.create_subscription(
            String,
            self.profile_topic,
            self.profile_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.delay_timer = self.create_timer(
            1.0 / delay_poll_rate_hz, self.publish_due_delayed_messages
        )

        self.reset_topic = self.get_parameter('reset_topic').get_parameter_value().string_value
        if self.reset_topic:
            self.reset_subscription = self.create_subscription(
                Bool, self.reset_topic, self.reset_callback, 10
            )

        self.get_logger().info(
            f'Perturbing {self.message_type_name}: {self.input_topic} -> {self.output_topic}, '
            f'control topic {self.control_topic}, profile topic {self.profile_topic}.'
        )

    def input_callback(self, msg):
        self.last_input_msg = deepcopy(msg)
        mode = self.current_mode()

        if self.active and mode == 'replay':
            replay_msg = self.next_replay_message()
            if replay_msg is not None:
                self.output_publisher.publish(replay_msg)
            else:
                self.output_publisher.publish(self._replay_hold_message())
            return

        self.record_replay_source_message(msg)

        if not self.active or mode == 'passthrough':
            self.output_publisher.publish(deepcopy(msg))
            return

        if mode == 'dropout':
            if self.dropout_active_now():
                return
            self.output_publisher.publish(deepcopy(msg))
            return

        if mode == 'delay':
            profile = self.effective_profile()
            delay_s = max(0.0, float(profile.get('delay_s', 0.0)))
            self.delay_queue.append((self._now_s() + delay_s, deepcopy(msg)))
            return

        transformed = self.transform_message(msg, mode)
        self.output_publisher.publish(transformed)

    def active_control_callback(self, msg: Bool):
        if self.active == msg.data:
            return

        self.active = msg.data
        self.active_started_at_s = self._now_s() if self.active else None
        self.dropout_passed_cycles = set()
        if self.active and self.current_mode() == 'replay':
            self.prepare_replay_sequence()
        self._configure_republish_timer()
        state = 'enabled' if self.active else 'disabled'
        self.get_logger().info(f'Topic perturbation {state}.')

        if not self.active and self.last_input_msg is not None:
            self.output_publisher.publish(deepcopy(self.last_input_msg))

    def reset_callback(self, msg: Bool):
        if not msg.data:
            return
        self.active = False
        self.active_started_at_s = None
        self.dropout_passed_cycles = set()
        self.replay_history.clear()
        self.replay_sequence.clear()
        self.replay_index = 0
        self.replay_empty_warned = False
        self.replay_capturing = False
        self.replay_capture_until_s = None
        self.delay_queue.clear()
        self.last_input_msg = None
        self.get_logger().info('Perturbation node state reset.')

    def _clear_replay_history(self):
        """Clear replay history — called when a new profile is loaded."""
        self.replay_history.clear()
        self.replay_sequence.clear()
        self.replay_index = 0
        self.replay_empty_warned = False

    def profile_callback(self, msg: String):
        self._update_profile(msg.data, log_invalid=True)
        # Clear replay history when a new profile is loaded so that only
        # messages from the current iteration's pre-activation phase are captured
        if self.current_mode() == 'replay':
            self.replay_history.clear()
            self.replay_sequence.clear()
            self.replay_index = 0
            self.replay_empty_warned = False

    def transform_message(self, msg, mode: str):
        if self.message_type is Twist:
            return self.transform_twist(msg, mode)

        self.get_logger().warn(
            f"Mode '{mode}' is only implemented for geometry_msgs/msg/Twist. Passing through."
        )
        return deepcopy(msg)

    def transform_twist(self, msg: Twist, mode: str) -> Twist:
        transformed = Twist()
        transformed.linear.x = msg.linear.x
        transformed.linear.y = msg.linear.y
        transformed.linear.z = msg.linear.z
        transformed.angular.x = msg.angular.x
        transformed.angular.y = msg.angular.y
        transformed.angular.z = msg.angular.z

        profile = self.effective_profile()
        if mode == 'passthrough':
            return transformed

        linear = self._apply_profile(
            [transformed.linear.x, transformed.linear.y, transformed.linear.z],
            profile,
            'linear',
            mode,
        )
        angular = self._apply_profile(
            [transformed.angular.x, transformed.angular.y, transformed.angular.z],
            profile,
            'angular',
            mode,
        )

        transformed.linear.x, transformed.linear.y, transformed.linear.z = linear
        transformed.angular.x, transformed.angular.y, transformed.angular.z = angular
        return transformed

    def _apply_profile(
        self, values: list[float], profile: dict[str, Any], prefix: str, mode: str
    ) -> list[float]:
        override = self._vector_or_none(profile.get(f'{prefix}_override'))
        minimum = self._vector_or_none(profile.get(f'{prefix}_min'))
        maximum = self._vector_or_none(profile.get(f'{prefix}_max'))

        if mode == 'override' and override is not None:
            result = override
        else:
            scale = self._vector_or_default(profile.get(f'{prefix}_scale'), [1.0, 1.0, 1.0])
            bias = self._vector_or_default(profile.get(f'{prefix}_bias'), [0.0, 0.0, 0.0])
            result = [value * factor for value, factor in zip(values, scale)]
            result = [value + delta for value, delta in zip(result, bias)]

        if minimum is not None:
            result = [max(value, lower) for value, lower in zip(result, minimum)]
        if maximum is not None:
            result = [min(value, upper) for value, upper in zip(result, maximum)]

        return result

    def _update_profile(self, profile_json: str, log_invalid: bool):
        try:
            parsed_profile = json.loads(profile_json) if profile_json else {'mode': 'passthrough'}
        except json.JSONDecodeError as exc:
            if log_invalid:
                self.get_logger().error(f'Invalid topic perturbation profile JSON: {exc}')
            return

        if not isinstance(parsed_profile, dict):
            if log_invalid:
                self.get_logger().error(
                    'Topic perturbation profile JSON must decode to an object.'
                )
            return

        self.current_profile = parsed_profile
        if self.active:
            self.active_started_at_s = self._now_s()
            self.dropout_passed_cycles = set()
            if self.current_mode() == 'replay':
                self.prepare_replay_sequence()
        if self.current_mode() != 'delay':
            self.delay_queue.clear()

        effective_profile = self.effective_profile()
        self.current_publish_rate_hz = float(
            effective_profile.get('publish_rate_hz', self.default_publish_rate_hz)
        )
        self._configure_republish_timer()
        self.get_logger().info(
            'Loaded topic perturbation profile '
            f'mode={effective_profile.get("mode", "passthrough")} '
            f'publish_rate_hz={self.current_publish_rate_hz:0.2f}.'
        )

    def publish_due_delayed_messages(self):
        if self.current_mode() != 'delay':
            return

        now = self._now_s()
        ready_messages = [item for item in self.delay_queue if item[0] <= now]
        self.delay_queue = [item for item in self.delay_queue if item[0] > now]
        for _, msg in ready_messages:
            self.output_publisher.publish(msg)

    def _configure_republish_timer(self):
        if self.republish_timer is not None:
            self.republish_timer.cancel()
            self.destroy_timer(self.republish_timer)
            self.republish_timer = None

        if not self.active:
            return

        mode = self.current_mode()
        publish_rate_hz = self.current_publish_rate_hz
        if mode == 'replay':
            return

        if publish_rate_hz <= 0.0:
            return

        self.republish_timer = self.create_timer(
            1.0 / publish_rate_hz, self.republish_last_message_callback
        )

    def republish_last_message_callback(self):
        if not self.active or self.last_input_msg is None:
            return

        mode = self.current_mode()
        if mode in ('delay', 'dropout', 'passthrough', 'replay'):
            return

        self.output_publisher.publish(self.transform_message(self.last_input_msg, mode))

    def effective_profile(self) -> dict[str, Any]:
        profile = dict(self.current_profile)
        profile_parameters = profile.pop('parameters', None)
        if isinstance(profile_parameters, dict):
            profile.update(profile_parameters)
        return profile

    def dropout_active_now(self) -> bool:
        profile = self.effective_profile()
        if 'dropout_duration_s' not in profile:
            return True

        if self.active_started_at_s is None:
            return False

        try:
            dropout_duration_s = float(profile.get('dropout_duration_s', 0.0))
            dropout_start_s = float(profile.get('dropout_start_s', 0.0))
        except (TypeError, ValueError):
            return True

        if dropout_duration_s <= 0.0:
            return False

        elapsed_s = self._now_s() - self.active_started_at_s
        if elapsed_s < dropout_start_s:
            return False

        cycle_index = int((elapsed_s - dropout_start_s) / dropout_duration_s)
        cycle_elapsed_s = (elapsed_s - dropout_start_s) - (cycle_index * dropout_duration_s)
        if cycle_elapsed_s <= 1e-6:
            return True
        if cycle_index not in self.dropout_passed_cycles:
            self.dropout_passed_cycles.add(cycle_index)
            return False
        return True

    def record_replay_source_message(self, msg):
        now = self._now_s()
        self.replay_history.append((now, deepcopy(msg)))

        profile = self.effective_profile()
        try:
            replay_window_s = float(profile.get('replay_window_s', 0.0))
        except (TypeError, ValueError):
            replay_window_s = 0.0
        history_horizon_s = max(60.0, replay_window_s)
        cutoff_s = now - history_horizon_s
        self.replay_history = [item for item in self.replay_history if item[0] >= cutoff_s]

    def prepare_replay_sequence(self):
        now = self._now_s()
        profile = self.effective_profile()
        replay_window_s = self.profile_float(profile, 'replay_window_s', 0.0)

        cutoff_s = now - replay_window_s if replay_window_s > 0.0 else None
        self.replay_sequence = [
            deepcopy(msg)
            for stamp_s, msg in self.replay_history
            if cutoff_s is None or stamp_s >= cutoff_s
        ]
        self.replay_index = 0
        self.replay_empty_warned = False
        self.replay_capturing = False
        self.replay_capture_until_s = None
        self.get_logger().info(
            f'Prepared fixed replay buffer with {len(self.replay_sequence)} message(s) '
            f'from last {replay_window_s:0.2f} s. Replaying same buffer for the full duration.'
        )

    def next_replay_message(self):
        if not self.replay_sequence:
            if not self.replay_empty_warned:
                self.get_logger().warn('Replay mode has no buffered messages. Holding last input.')
                self.replay_empty_warned = True
            return None

        replay_msg = deepcopy(self.replay_sequence[self.replay_index])
        if self.replay_index < len(self.replay_sequence) - 1:
            self.replay_index += 1
        return replay_msg

    def _replay_hold_message(self):
        if self.replay_sequence:
            return deepcopy(self.replay_sequence[-1])
        if self.last_input_msg is not None:
            return deepcopy(self.last_input_msg)
        return None

    def current_mode(self) -> str:
        return str(self.effective_profile().get('mode', 'passthrough')).strip().lower()

    @staticmethod
    def profile_float(profile: dict[str, Any], name: str, default: float) -> float:
        try:
            return float(profile.get(name, default))
        except (TypeError, ValueError):
            return default

    def _now_s(self) -> float:
        try:
            ns = self.get_clock().now().nanoseconds
            if self.use_sim_time or ns != 0:
                return float(ns) / 1e9
        except Exception:
            pass
        return time.monotonic()

    @staticmethod
    def _vector_or_default(value: Any, default: list[float]) -> list[float]:
        normalized = TopicPerturbationNode._vector_or_none(value)
        return normalized if normalized is not None else default

    @staticmethod
    def _vector_or_none(value: Any) -> list[float] | None:
        if value is None:
            return None
        if isinstance(value, (int, float)):
            return [float(value)] * 3
        if isinstance(value, (list, tuple)):
            if len(value) == 1:
                return [float(value[0])] * 3
            if len(value) == 3:
                return [float(item) for item in value]
        return None

    def destroy_node(self):
        if self.republish_timer is not None:
            self.republish_timer.cancel()
            self.destroy_timer(self.republish_timer)
            self.republish_timer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TopicPerturbationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('User requested shutdown with SIGINT.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
