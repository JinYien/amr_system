#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from amr_message.msg import PidCommand, PidGain, Teensy
from amr_message.qos import QOS_PROFILE
from amr_tuner.hardware import Teensy40
from amr_tuner.protocol import RxMessageType
from amr_tuner.settings import Settings, load_settings


class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")
        self.get_logger().info("Initializing ...")
        config = self.declare_parameter("config", "").get_parameter_value().string_value

        self.settings: Settings = load_settings(config)
        self.teensy = Teensy40(self.settings.serial)

        self.left_velocity_command = 0.0
        self.right_velocity_command = 0.0

        self.get_logger().info("Publishing & subscribing ...")
        topics = self.settings.topics

        self.teensy_publisher = self.create_publisher(Teensy, topics.teensy, QOS_PROFILE)

        self.create_subscription(PidGain, topics.gain, self.gain_callback, QOS_PROFILE)
        self.create_subscription(PidCommand, topics.command, self.command_callback, QOS_PROFILE)

        self.get_logger().info("Running process ...")
        rates = self.settings.rates

        self.create_timer(1.0 / rates.drive_command, self.drive_motor_loop)
        self.create_timer(1.0 / rates.serial_read, self.teensy_loop)

    def gain_callback(self, message: PidGain):
        self.teensy.apply_pid(message.p_gain, message.i_gain, message.d_gain)

    def command_callback(self, message: PidCommand):
        self.left_velocity_command = message.left_velocity
        self.right_velocity_command = message.right_velocity

    def drive_motor_loop(self):
        self.teensy.set_motor_speed(self.left_velocity_command, self.right_velocity_command)

    def teensy_loop(self):
        try:
            message_type, payload = self.teensy.read_packet()
        except Exception as error:
            self.get_logger().error(str(error))
            return

        if message_type != RxMessageType.DATA:
            return

        sample = payload[-1]
        message = Teensy()
        message.right_velocity = float(sample[3])
        message.left_velocity = float(sample[4])
        self.teensy_publisher.publish(message)


def main():
    rclpy.init()
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
