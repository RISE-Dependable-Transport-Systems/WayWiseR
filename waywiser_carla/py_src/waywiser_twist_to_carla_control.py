#!/usr/bin/env python3


import math

from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0


class WaywiserTwistToCarlaControl(Node):
    def __init__(self):
        super().__init__('waywiser_twist_to_carla_control')

        # Declare parameters
        self.declare_parameter('speed_control_threshold', 0.1)
        self.declare_parameter('max_steering_angle', 0.7)
        self.declare_parameter('max_throttle', 1.0)
        self.declare_parameter('max_brake', 1.0)
        self.declare_parameter('speed_control_kp', 1.0)
        self.declare_parameter('speed_control_ki', 0.1)
        self.declare_parameter('speed_control_kd', 0.05)
        self.declare_parameter('ego_vehicle_role_name', 'ego_vehicle')
        self.declare_parameter('speed_control_loop_rate', 30)

        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('standstill_velocity_threshold', 0.01)
        self.declare_parameter('max_angular_velocity', 0.5)

        # Get parameters
        self.speed_control_threshold = self.get_parameter('speed_control_threshold').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.max_brake = self.get_parameter('max_brake').value
        self.ego_vehicle_role_name = (
            self.get_parameter('ego_vehicle_role_name').get_parameter_value().string_value
        )
        self.speed_control_loop_rate = self.get_parameter('speed_control_loop_rate').value

        self.wheelbase = self.get_parameter('wheelbase').value
        self.standstill_velocity_threshold = self.get_parameter(
            'standstill_velocity_threshold'
        ).value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value

        # Initialize PID controller
        speed_control_kp = self.get_parameter('speed_control_kp').value
        speed_control_ki = self.get_parameter('speed_control_ki').value
        speed_control_kd = self.get_parameter('speed_control_kd').value

        self.pid_speed_controller = PIDController(
            speed_control_kp, speed_control_ki, speed_control_kd
        )

        # Initialize variables
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.target_angular_speed = 0.0
        self.last_control_time = self.get_clock().now()
        self.speed_precision_digits = 4

        # Set up QoS profile
        subscriber_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create subscribers
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel_out', self.twist_callback, subscriber_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/{}/odometry'.format(self.ego_vehicle_role_name),
            self.odom_callback,
            subscriber_qos,
        )

        # Create publisher
        self.control_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/{}/vehicle_control_cmd'.format(self.ego_vehicle_role_name),
            publisher_qos,
        )

        # Create timer for control loop
        self.timer = self.create_timer(1.0 / self.speed_control_loop_rate, self.control_loop)

    def twist_callback(self, msg):
        self.target_speed = round(msg.linear.x, self.speed_precision_digits)
        if abs(self.target_speed) < self.speed_control_threshold:
            self.target_speed = 0.0
        self.target_angular_speed = msg.angular.z

    def odom_callback(self, msg):
        self.current_speed = round(msg.twist.twist.linear.x, self.speed_precision_digits)

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time

        reverse = False
        if abs(self.target_speed) > 0:
            # Compute speed error
            speed_error = self.target_speed - self.current_speed

            # Compute throttle and brake
            speed_controller_signal = self.pid_speed_controller.compute(speed_error, dt)
            if self.target_speed < 0:
                reverse = True
                speed_controller_signal = -speed_controller_signal

            # Convert to throttle and brake
            if speed_controller_signal > 0:
                throttle = min(speed_controller_signal, self.max_throttle)
                brake = 0.0
            else:
                throttle = 0.0
                brake = min(-speed_controller_signal, self.max_brake)
        else:
            throttle = 0.0
            brake = self.max_brake
            self.pid_speed_controller.reset()

        # Compute steering
        if abs(self.current_speed) > self.standstill_velocity_threshold:
            target_steering_curvature = -(self.target_angular_speed / abs(self.current_speed))
            target_steering = math.atan(self.wheelbase * target_steering_curvature)

            target_steering = (
                max(-self.max_steering_angle, min(self.max_steering_angle, target_steering))
                / self.max_steering_angle
            )  # Normalize to [-1, 1]
        else:
            target_steering = max(
                -1.0, min(1.0, -self.target_angular_speed / self.max_angular_velocity)
            )

        if self.target_speed <= 0:
            target_steering = -target_steering

        # Create and publish control message
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = float(throttle)
        control_msg.brake = float(brake)
        control_msg.steer = float(target_steering)
        control_msg.reverse = reverse

        self.control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaywiserTwistToCarlaControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
