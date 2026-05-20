#!/usr/bin/env python3

import json
from copy import deepcopy
from typing import Any

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from waywiser_py.waywiser_utils import RELIABLE_TRANSIENT_LOCAL_QOS


class CommunicationChannelEmulator(Node):
    def __init__(self):
        super().__init__('communication_channel_emulator')

        self.declare_parameter('input_topic', 'teleop_mux_vel')
        self.declare_parameter('output_topic', 'teleop_mux_vel_compromised')
        self.declare_parameter('attack_control_topic', 'scenario_3b/attack_enabled')
        self.declare_parameter('attack_profile_topic', 'scenario_3b/attack_profile')
        self.declare_parameter('default_profile_json', '{"mode": "passthrough"}')
        self.declare_parameter('start_active', False)
        self.declare_parameter('default_publish_rate_hz', 0.0)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.attack_control_topic = (
            self.get_parameter('attack_control_topic').get_parameter_value().string_value
        )
        self.attack_profile_topic = (
            self.get_parameter('attack_profile_topic').get_parameter_value().string_value
        )
        self.default_publish_rate_hz = (
            self.get_parameter('default_publish_rate_hz').get_parameter_value().double_value
        )
        self.attack_active = self.get_parameter('start_active').get_parameter_value().bool_value
        self.current_profile: dict[str, Any] = {'mode': 'passthrough'}
        self.last_input_msg: Twist | None = None
        self.republish_timer = None
        self.current_publish_rate_hz = 0.0

        default_profile_json = (
            self.get_parameter('default_profile_json').get_parameter_value().string_value
        )
        self._update_profile(default_profile_json, log_invalid=False)

        self.output_publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.input_callback, 10)
        self.create_subscription(
            Bool,
            self.attack_control_topic,
            self.attack_control_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )
        self.create_subscription(
            String,
            self.attack_profile_topic,
            self.attack_profile_callback,
            RELIABLE_TRANSIENT_LOCAL_QOS,
        )

        self.get_logger().info(
            f'Listening on {self.input_topic}, publishing to {self.output_topic}, '
            f'control topic {self.attack_control_topic}.'
        )

    def input_callback(self, msg: Twist):
        self.last_input_msg = deepcopy(msg)
        self.output_publisher.publish(self.transform_twist(msg))

    def attack_control_callback(self, msg: Bool):
        if self.attack_active == msg.data:
            return

        self.attack_active = msg.data
        state = 'enabled' if self.attack_active else 'disabled'
        self.get_logger().info(f'Attack state {state}.')

        if not self.attack_active and self.last_input_msg is not None:
            self.output_publisher.publish(self.transform_twist(self.last_input_msg))

    def attack_profile_callback(self, msg: String):
        self._update_profile(msg.data, log_invalid=True)

    def transform_twist(self, msg: Twist) -> Twist:
        transformed = Twist()
        transformed.linear.x = msg.linear.x
        transformed.linear.y = msg.linear.y
        transformed.linear.z = msg.linear.z
        transformed.angular.x = msg.angular.x
        transformed.angular.y = msg.angular.y
        transformed.angular.z = msg.angular.z

        if not self.attack_active:
            return transformed

        profile = self._effective_profile()
        mode = str(profile.get('mode', 'compose')).lower()
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
                self.get_logger().error(f'Invalid attack profile JSON: {exc}')
            return

        if not isinstance(parsed_profile, dict):
            if log_invalid:
                self.get_logger().error('Attack profile JSON must decode to an object.')
            return

        self.current_profile = parsed_profile
        effective_profile = self._effective_profile()
        self.current_publish_rate_hz = float(
            effective_profile.get('publish_rate_hz', self.default_publish_rate_hz)
        )
        self._configure_republish_timer()
        self.get_logger().info(
            f'Loaded attack profile mode={effective_profile.get("mode", "compose")} '
            f'publish_rate_hz={self.current_publish_rate_hz:0.2f}.'
        )

    def _effective_profile(self) -> dict[str, Any]:
        profile = dict(self.current_profile)
        profile_parameters = profile.pop('parameters', None)
        if isinstance(profile_parameters, dict):
            profile.update(profile_parameters)
        return profile

    def _configure_republish_timer(self):
        if self.republish_timer is not None:
            self.republish_timer.cancel()
            self.destroy_timer(self.republish_timer)
            self.republish_timer = None

        if self.current_publish_rate_hz <= 0.0:
            return

        self.republish_timer = self.create_timer(
            1.0 / self.current_publish_rate_hz, self.republish_last_twist_callback
        )

    def republish_last_twist_callback(self):
        if not self.attack_active or self.last_input_msg is None:
            return

        self.output_publisher.publish(self.transform_twist(self.last_input_msg))

    @staticmethod
    def _vector_or_default(value: Any, default: list[float]) -> list[float]:
        normalized = CommunicationChannelEmulator._vector_or_none(value)
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


def main(args=None):
    rclpy.init(args=args)
    node = CommunicationChannelEmulator()
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