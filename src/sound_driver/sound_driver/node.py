import os
import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from custom_message.msg import Sound
from sound_driver.beep import BeepController
from sound_driver.settings import Settings, load_settings
from sound_driver.sound import SoundPlayer

BEEP_LOOP_PERIOD = 0.05


class SoundNode(Node):
    def __init__(self):
        super().__init__("sound_node")

        # ======================================================
        self.get_logger().info("Initializing node ...")
        # ======================================================
        config_path = self.declare_parameter("config_path", "").get_parameter_value().string_value
        self.settings: Settings = load_settings(config_path)

        directory = os.path.join(get_package_share_directory("sound_driver"), "wav")
        self.player = SoundPlayer(self.settings.beep, directory)
        self.beep_controller = BeepController(self.settings.beep)
        self.obstacle_distance = 0.0
        self.playback_error_logged = False

        # ======================================================
        self.get_logger().info("Publishing & subscribing topics ...")
        # ======================================================
        qos = qos_profile_sensor_data

        self.create_subscription(Sound, self.settings.topics.sound_subscriber, self.sound_callback, qos)

        # ======================================================
        self.get_logger().info("Running process ...")
        # ======================================================
        self.create_timer(BEEP_LOOP_PERIOD, self.beep_loop)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def sound_callback(self, message: Sound):
        if message.path:
            self.play_clip(message.path)
        if message.object == "beep":
            self.play_test_beep()
        elif message.object == "chime":
            self.play_chime()
        elif message.object:
            self.play_clip(message.object)
        if not message.path and not message.object:
            self.obstacle_distance = float(message.obstacle_distance)

    def play_test_beep(self):
        try:
            self.player.play_single_beep()
        except Exception:
            self.log_playback_error()

    def play_chime(self):
        try:
            self.player.play_chime()
        except Exception:
            self.log_playback_error()

    def play_clip(self, name: str):
        try:
            if not self.player.play_clip(name):
                self.get_logger().warning(f"Sound clip '{name}' not found in the wav folder, nothing played")
        except Exception:
            self.log_playback_error()

    def beep_loop(self):
        try:
            self.player.update_beep(self.beep_controller.pattern(self.obstacle_distance), self.now())
        except Exception:
            self.log_playback_error()

    def log_playback_error(self):
        if not self.playback_error_logged:
            self.get_logger().error(f"Sound playback with aplay failed, no audio output\n{traceback.format_exc()}")
            self.playback_error_logged = True


def main():
    rclpy.init()
    node = SoundNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
