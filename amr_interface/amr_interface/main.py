#!/usr/bin/env python3

import threading
from typing import Callable, Optional
import rclpy
from rclpy.node import Node
from amr_message.msg import Command, Control, State
from amr_message.qos import QOS_PROFILE
from amr_interface.server import resolve_web_dir, run_web_server
from amr_interface.settings import Settings, load_settings


class InterfaceNode(Node):
    def __init__(self):
        super().__init__("interface_node")
        self.get_logger().info("Initializing ...")
        config = self.declare_parameter("config", "").get_parameter_value().string_value
        self.settings: Settings = load_settings(config)

        self.mode = "manual"
        self.authority = ""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.middle_mode = ""
        self.middle_action = ""

        self.topic_callback: Optional[Callable[[dict], None]] = None

        self.get_logger().info("Publishing & subscribing ...")
        topics = self.settings.topics

        self.command_publisher = self.create_publisher(Command, topics.command_publisher, QOS_PROFILE)
        self.control_publisher = self.create_publisher(Control, topics.control_publisher, QOS_PROFILE)

        self.create_subscription(State, topics.state_subscriber, self.state_callback, QOS_PROFILE)

        self.get_logger().info("Running process ...")
        self.create_timer(1.0 / self.settings.rates.command_publish, self.publish_message)

    def state_callback(self, message: State):
        self.broadcast_topic(
            "state",
            {
                "position_x": float(message.position_x),
                "position_y": float(message.position_y),
                "orientation": float(message.orientation),
            },
        )

    def broadcast_topic(self, topic: str, data: dict):
        if self.topic_callback:
            self.topic_callback({"type": "topic", "topic": topic, "data": data})

    def set_topic_callback(self, callback: Callable[[dict], None]):
        self.topic_callback = callback

    def publish_message(self):
        command = Command()
        command.linear_velocity = self.linear_velocity
        command.angular_velocity = self.angular_velocity
        command.middle_mode = self.middle_mode
        command.middle_action = self.middle_action
        self.command_publisher.publish(command)

        control = Control()
        control.mode = self.mode
        control.authority = self.authority if self.mode == "auto" else ""
        self.control_publisher.publish(control)

        self.middle_mode = ""
        self.middle_action = ""

    def handle_event(self, event: dict):
        event_type = event.get("type")
        vocabulary = self.settings.vocabulary
        limits = self.settings.limits

        if event_type == "axes":
            self.linear_velocity = self.clamp(event.get("linear", 0.0), -limits.max_linear_velocity, limits.max_linear_velocity)
            self.angular_velocity = self.clamp(
                event.get("angular", 0.0), -limits.max_angular_velocity, limits.max_angular_velocity
            )
            return

        if event_type == "mode":
            requested = event.get("value", "manual")
            if requested in vocabulary.modes and requested != self.mode:
                self.mode = requested
                self.authority = ""
            return

        if event_type == "authority":
            requested = event.get("value", "")
            if self.mode == "auto" and requested in vocabulary.authorities:
                self.authority = requested
            return

        if event_type == "middle":
            requested_mode = event.get("mode", "")
            requested_action = event.get("action", "")
            if requested_mode in vocabulary.middle_modes:
                self.middle_mode = requested_mode
            if requested_action in vocabulary.middle_actions:
                self.middle_action = requested_action

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, float(value)))


def main():
    rclpy.init()
    node = InterfaceNode()
    web_dir = resolve_web_dir()

    server_thread = threading.Thread(target=run_web_server, args=(node, web_dir), daemon=True)
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
