#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from amr_message.msg import Command, Control, Teensy
from amr_message.qos import QOS_PROFILE
from amr_system.kinematics import forward_kinematics
from amr_system.settings import Settings, load_settings


class CommandNode(Node):
    def __init__(self):
        super().__init__("command_node")

        self.get_logger().info("Initializing ...")
        config = self.declare_parameter("config", "").get_parameter_value().string_value
        robot = self.declare_parameter("robot", "").get_parameter_value().string_value
        self.settings: Settings = load_settings(config, robot)

        self.mode = "manual"
        self.authority = ""

        self.user_command_left_velocity = 0.0
        self.user_command_right_velocity = 0.0
        self.robot_command_left_velocity = 0.0
        self.robot_command_right_velocity = 0.0
        self.command_middle_mode = ""
        self.command_middle_action = ""

        self.raw_force = 0.0
        self.raw_azimuth_angle = 0.0

        self.wheel = self.settings.wheel

        force = self.settings.force
        self.force_deadzone = force.deadzone
        self.force_clamp_min = force.clamp_min
        self.force_clamp_max = force.clamp_max
        self.force_useful_range = force.useful_range
        self.force_low_pass_alpha = force.low_pass_alpha
        self.force_filtered = 0.0

        linear = self.settings.linear
        self.linear_forward_gain = linear.forward_gain
        self.linear_reverse_gain = linear.reverse_gain
        self.linear_max = linear.max
        self.linear_min = linear.min
        self.max_linear_acceleration = linear.max_acceleration
        self.max_linear_deceleration = linear.max_deceleration

        self.control_period = self.settings.loop.period
        self.max_angular_acceleration = self.settings.angular.max_acceleration
        self.linear_velocity_command = 0.0
        self.angular_velocity_command = 0.0

        self.get_logger().info("Publishing & subscribing ...")
        topics = self.settings.topics

        self.command_publisher = self.create_publisher(Command, topics.command_publisher, QOS_PROFILE)

        self.create_subscription(Control, topics.control_subscriber, self.control_callback, QOS_PROFILE)
        self.create_subscription(Command, topics.joystick_subscriber, self.joystick_command_callback, QOS_PROFILE)
        self.create_subscription(Command, topics.robot_subscriber, self.robot_command_callback, QOS_PROFILE)
        self.create_subscription(Teensy, topics.teensy_subscriber, self.user_command_callback, QOS_PROFILE)

        self.get_logger().info("Running control loop ...")
        self.create_timer(self.control_period, self.publish_command)

    def control_callback(self, message: Control):
        self.mode = message.mode
        self.authority = message.authority

    def joystick_command_callback(self, message: Command):
        if self.mode == "manual":
            left_velocity, right_velocity = forward_kinematics(message.linear_velocity, message.angular_velocity, self.wheel)

            command_message = Command()
            command_message.left_velocity = float(np.rad2deg(left_velocity))
            command_message.right_velocity = float(np.rad2deg(right_velocity))
            command_message.middle_mode = message.middle_mode
            command_message.middle_action = message.middle_action
            self.command_publisher.publish(command_message)

    def robot_command_callback(self, message: Command):
        self.robot_command_left_velocity = message.left_velocity
        self.robot_command_right_velocity = message.right_velocity
        self.command_middle_mode = message.middle_mode
        self.command_middle_action = message.middle_action

    def user_command_callback(self, message: Teensy):
        self.raw_force = float(message.force)
        self.raw_azimuth_angle = float(message.azimuth_angle)

    def publish_command(self):
        self.shape_user_command()

        if self.mode == "auto":
            command_message = Command()
            command_message.middle_mode = self.command_middle_mode
            command_message.middle_action = self.command_middle_action

            if self.authority == "user":
                command_message.left_velocity = self.user_command_left_velocity
                command_message.right_velocity = self.user_command_right_velocity
            elif self.authority == "robot":
                command_message.left_velocity = self.robot_command_left_velocity
                command_message.right_velocity = self.robot_command_right_velocity
            elif self.authority == "assist":  # TODO
                command_message.left_velocity = 0.0
                command_message.right_velocity = 0.0

            self.command_publisher.publish(command_message)

    @staticmethod
    def soft_deadzone(value: float, threshold: float) -> float:
        if value > threshold:
            return value - threshold
        if value < -threshold:
            return value + threshold
        return 0.0

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def rate_limit(self, target: float, current: float, max_acceleration: float) -> float:
        max_step = max_acceleration * self.control_period
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def rate_limit_linear(self, target: float, current: float) -> float:
        delta = target - current
        decelerating = (current > 0.0 and delta < 0.0) or (current < 0.0 and delta > 0.0)
        max_acceleration = self.max_linear_deceleration if decelerating else self.max_linear_acceleration
        max_step = max_acceleration * self.control_period
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def shape_user_command(self):
        self.force_filtered += self.force_low_pass_alpha * (self.raw_force - self.force_filtered)

        force = self.clamp(self.force_filtered, self.force_clamp_min, self.force_clamp_max)
        force = self.soft_deadzone(force, self.force_deadzone)

        useful = self.force_useful_range - self.force_deadzone
        if useful <= 0.0:
            normalized = 0.0
        else:
            normalized = force / useful

        shape = float(np.tanh(normalized))
        gain = self.linear_forward_gain if shape >= 0.0 else self.linear_reverse_gain
        linear_target = self.clamp(gain * shape, self.linear_min, self.linear_max)

        angular_target = -np.rad2deg(self.raw_azimuth_angle)

        self.linear_velocity_command = self.rate_limit_linear(linear_target, self.linear_velocity_command)
        self.linear_velocity_command = self.clamp(self.linear_velocity_command, self.linear_min, self.linear_max)
        self.angular_velocity_command = self.rate_limit(
            angular_target, self.angular_velocity_command, self.max_angular_acceleration
        )

        left_velocity, right_velocity = forward_kinematics(
            self.linear_velocity_command, self.angular_velocity_command, self.wheel
        )
        self.user_command_left_velocity = float(left_velocity)
        self.user_command_right_velocity = float(right_velocity)


def main():
    rclpy.init()
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
