#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from amr_message.msg import Command, State, Teensy
from amr_message.qos import QOS_PROFILE
from amr_teensy.hardware import Teensy40
from amr_teensy.model import euler_to_quaternion, inverse_kinematics, torque_to_wrench, update_pose
from amr_teensy.protocol import STATE_VARIABLES, RxMessageType
from amr_teensy.settings import Settings, load_settings


class TeensyNode(Node):
    def __init__(self):
        super().__init__("teensy_node")
        self.get_logger().info("Initializing ...")
        config = self.declare_parameter("config", "").get_parameter_value().string_value
        robot = self.declare_parameter("robot", "").get_parameter_value().string_value

        self.settings: Settings = load_settings(config, robot)
        self.teensy = Teensy40(self.settings.serial, self.settings.drive, self.settings.handle)

        self.pose = np.zeros(3)
        self.last_time = None

        self.left_velocity_command = 0.0
        self.right_velocity_command = 0.0

        self.handle_mode = None
        self.handle_action = None
        self.handle_action_start = None
        self.pulse_step = 0

        self.emergency_stop = False
        self.emergency_stop_log = False

        self.get_logger().info("Publishing & subscribing ...")
        topics = self.settings.topics

        self.teensy_publisher = self.create_publisher(Teensy, topics.teensy_publisher, QOS_PROFILE)
        self.state_publisher = self.create_publisher(State, topics.state_publisher, QOS_PROFILE)
        self.odometry_publisher = self.create_publisher(Odometry, topics.odometry_publisher, QOS_PROFILE)

        self.create_subscription(Command, topics.command_subscriber, self.command_callback, QOS_PROFILE)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Running process ...")
        rates = self.settings.rates

        self.create_timer(1.0 / rates.serial_read, self.teensy_loop)
        self.create_timer(1.0 / rates.drive_command, self.drive_motor_loop)
        self.create_timer(1.0 / rates.handle_command, self.handle_motor_loop)

    def command_callback(self, message: Command):
        self.left_velocity_command = message.left_velocity
        self.right_velocity_command = message.right_velocity

        if message.middle_mode and self.handle_mode is None:
            if message.middle_mode == "pulse":
                self.handle_mode = "pulse"
                self.handle_action = None
            elif message.middle_mode == "nudge" and message.middle_action:
                self.handle_mode = "nudge"
                self.handle_action = message.middle_action

    def drive_motor_loop(self):
        if self.emergency_stop:
            self.left_velocity_command = 0.0
            self.right_velocity_command = 0.0
            return
        self.teensy.set_motor_speed(self.left_velocity_command, self.right_velocity_command)

    def handle_motor_loop(self):
        if self.emergency_stop:
            self.teensy.set_cybergear_free()
            return
        if self.handle_mode == "nudge":
            self.nudge_handle()
        elif self.handle_mode == "pulse":
            self.pulse_handle()

    def nudge_handle(self):
        now = self.get_clock().now()
        nudge = self.settings.handle.nudge

        if self.handle_action is not None:
            sign = -1.0 if self.handle_action == "left" else 1.0
            self.teensy.set_cybergear_torque(sign * nudge.torque)
            self.handle_action = None
            self.handle_action_start = now
            return

        elapsed_seconds = (now - self.handle_action_start).nanoseconds / 1e9
        if elapsed_seconds > nudge.hold_time:
            self.teensy.set_cybergear_torque(0.0)
            self.handle_mode = None

    def pulse_handle(self):
        now = self.get_clock().now()
        pulse = self.settings.handle.pulse

        if self.handle_action is None:
            self.handle_action = True
            self.handle_action_start = now
            self.pulse_step = 0
            return

        elapsed_seconds = (now - self.handle_action_start).nanoseconds / 1e9
        if elapsed_seconds < pulse.hold_time:
            return

        if self.pulse_step < len(pulse.sequence):
            self.teensy.set_cybergear_torque(pulse.sequence[self.pulse_step] * pulse.torque)
            self.pulse_step += 1
            self.handle_action_start = now
        else:
            self.handle_mode = None
            self.handle_action = None

    def teensy_loop(self):
        try:
            message_type, payload = self.teensy.read_packet()
        except Exception as error:
            self.get_logger().error(str(error))
            return

        if message_type != RxMessageType.DATA:
            return

        sample = payload[-1]
        if len(sample) != len(STATE_VARIABLES):
            return

        self.update_emergency_stop(bool(sample[0]))

        self.publish_teensy(sample)
        self.publish_state(sample)

    def update_emergency_stop(self, active: bool):
        self.emergency_stop = active
        if active and not self.emergency_stop_log:
            self.get_logger().warn("Emergency stop triggered!")
            self.emergency_stop_log = True
        if not active:
            self.emergency_stop_log = False

    def publish_teensy(self, sample):
        message = Teensy()
        message.emergency_stop = int(sample[0])
        message.right_angle = float(sample[1])
        message.left_angle = float(sample[2])
        message.right_velocity = float(sample[3])
        message.left_velocity = float(sample[4])
        message.right_torque = float(sample[5])
        message.left_torque = float(sample[6])
        message.rotation_speed = float(sample[7])
        message.azimuth_angle = float(sample[8])
        message.force = float(sample[9])
        self.teensy_publisher.publish(message)

    def publish_state(self, sample):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        elapsed_seconds = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if elapsed_seconds >= self.settings.odometry.max_timestep:
            return

        wheel = self.settings.wheel
        left_wheel_velocity = np.deg2rad(sample[4])
        right_wheel_velocity = np.deg2rad(sample[3])

        linear_velocity, angular_velocity = inverse_kinematics(left_wheel_velocity, right_wheel_velocity, wheel)
        linear_force, rotational_moment = torque_to_wrench(sample[6], sample[5], wheel)
        self.pose = update_pose(self.pose, elapsed_seconds, left_wheel_velocity, right_wheel_velocity, wheel)
        quaternion_x, quaternion_y, quaternion_z, quaternion_w = euler_to_quaternion(0.0, 0.0, self.pose[2])

        state = State()
        state.position_x = float(self.pose[0])
        state.position_y = float(self.pose[1])
        state.orientation = float(np.rad2deg(self.pose[2]))
        state.linear_velocity = float(linear_velocity)
        state.angular_velocity = float(angular_velocity)
        state.linear_force = float(linear_force)
        state.rotational_moment = float(rotational_moment)
        self.state_publisher.publish(state)

        frames = self.settings.frames
        odometry = Odometry()
        odometry.header.stamp = now.to_msg()
        odometry.header.frame_id = frames.odometry
        odometry.child_frame_id = frames.base
        odometry.pose.pose.position.x = float(self.pose[0])
        odometry.pose.pose.position.y = float(self.pose[1])
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = quaternion_x
        odometry.pose.pose.orientation.y = quaternion_y
        odometry.pose.pose.orientation.z = quaternion_z
        odometry.pose.pose.orientation.w = quaternion_w
        odometry.twist.twist.linear.x = float(linear_velocity)
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.angular.z = float(angular_velocity)
        self.odometry_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = frames.odometry
        transform.child_frame_id = frames.base
        transform.transform.translation.x = float(self.pose[0])
        transform.transform.translation.y = float(self.pose[1])
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion_x
        transform.transform.rotation.y = quaternion_y
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = TeensyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
