#!/usr/bin/env python3

import sys
import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from rclpy.node import Node
from amr_message.msg import PidCommand, PidGain, Teensy
from amr_message.qos import QOS_PROFILE
from amr_tuner.gui.window import MainWindow
from amr_tuner.settings import Settings, load_settings


class TunerNode(Node):
    def __init__(self, motor_callback=None):
        super().__init__("tuner_node")
        self.get_logger().info("Initializing ...")
        config = self.declare_parameter("config", "").get_parameter_value().string_value

        self.settings: Settings = load_settings(config)
        self.motor_callback = motor_callback

        self.get_logger().info("Publishing & subscribing ...")
        topics = self.settings.topics

        self.gain_publisher = self.create_publisher(PidGain, topics.gain, QOS_PROFILE)
        self.command_publisher = self.create_publisher(PidCommand, topics.command, QOS_PROFILE)

        self.create_subscription(Teensy, topics.teensy, self.teensy_callback, QOS_PROFILE)

    def teensy_callback(self, data: Teensy):
        if self.motor_callback:
            self.motor_callback(data.right_velocity, data.left_velocity)

    def send_pid_gain(self, p_gain: float, i_gain: float, d_gain: float):
        message = PidGain()
        message.p_gain = float(p_gain)
        message.i_gain = float(i_gain)
        message.d_gain = float(d_gain)
        self.gain_publisher.publish(message)

    def send_velocity_command(self, left_velocity: float, right_velocity: float):
        message = PidCommand()
        message.left_velocity = float(left_velocity)
        message.right_velocity = float(right_velocity)
        self.command_publisher.publish(message)


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    ros_node = TunerNode()
    window = MainWindow(ros_node.settings)
    window.ros_node = ros_node
    ros_node.motor_callback = window.motor_data
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    spin_timer.start(ros_node.settings.rates.ros_spin_interval)

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
