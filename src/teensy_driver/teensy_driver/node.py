import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from custom_message.msg import DriveCommand, HandleCommand, Teensy
from teensy_driver.hardware import Teensy40
from teensy_driver.handle import HandleController
from teensy_driver.protocol import TELEMETRY_FIELDS, RxMessageType
from teensy_driver.settings import Settings, load_settings


class TeensyNode(Node):
    def __init__(self):
        super().__init__("teensy_node")

        # ======================================================
        self.get_logger().info("Initializing node ...")
        # ======================================================
        config_path = self.declare_parameter("config_path", "").get_parameter_value().string_value
        self.settings: Settings = load_settings(config_path)

        self.teensy = Teensy40(self.settings.serial, self.settings.motor)
        self.left_velocity_command = 0.0
        self.right_velocity_command = 0.0
        middle = self.settings.motor.middle
        self.handle_controller = HandleController(middle.nudge, middle.pulse)
        self.emergency_stop = False
        self.last_drive_command_time = None
        self.watchdog_warning_logged = False

        # ======================================================
        self.get_logger().info("Publishing & subscribing topics ...")
        # ======================================================
        topics = self.settings.topics
        qos = qos_profile_sensor_data

        self.telemetry_publisher = self.create_publisher(Teensy, topics.telemetry_publisher, qos)

        self.drive_command_subscriber = self.create_subscription(
            DriveCommand, topics.drive_command_subscriber, self.drive_command_callback, qos
        )
        self.handle_command_subscriber = self.create_subscription(
            HandleCommand, topics.handle_command_subscriber, self.handle_command_callback, qos
        )

        # ======================================================
        self.get_logger().info("Running process ...")
        # ======================================================
        rates = self.settings.rates

        self.create_timer(1.0 / rates.serial_read, self.read_serial_loop)
        self.create_timer(1.0 / rates.drive_command, self.drive_motor_loop)
        self.create_timer(1.0 / rates.handle_command, self.handle_motor_loop)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def drive_command_callback(self, message: DriveCommand):
        self.left_velocity_command = message.left_velocity
        self.right_velocity_command = message.right_velocity
        self.last_drive_command_time = self.now()
        self.watchdog_warning_logged = False

    def drive_command_expired(self) -> bool:
        if self.last_drive_command_time is None:
            return True
        return self.now() - self.last_drive_command_time > self.settings.safety.command_timeout

    def handle_command_callback(self, message: HandleCommand):
        if message.mode == "nudge" and message.action:
            self.handle_controller.request_nudge(message.action)
        elif message.mode == "pulse":
            self.handle_controller.request_pulse()

    def drive_motor_loop(self):
        if self.emergency_stop:
            self.left_velocity_command = 0.0
            self.right_velocity_command = 0.0
        elif self.drive_command_expired():
            if (self.left_velocity_command or self.right_velocity_command) and not self.watchdog_warning_logged:
                self.get_logger().warning(
                    f"No drive command for more than {self.settings.safety.command_timeout}s "
                    f"in drive_motor_loop, stopping the wheels"
                )
                self.watchdog_warning_logged = True
            self.left_velocity_command = 0.0
            self.right_velocity_command = 0.0
        try:
            self.teensy.set_drive_speed(self.left_velocity_command, self.right_velocity_command)
        except Exception:
            self.get_logger().error(f"Serial write to the Teensy failed in drive_motor_loop\n{traceback.format_exc()}")

    def handle_motor_loop(self):
        try:
            if self.emergency_stop:
                self.teensy.free_handle_motor()
                return
            torque = self.handle_controller.update(self.now())
            if torque is not None:
                self.teensy.set_handle_torque(torque)
        except Exception:
            self.get_logger().error(f"Serial write to the Teensy failed in handle_motor_loop\n{traceback.format_exc()}")

    def read_serial_loop(self):
        try:
            message_type, payload = self.teensy.read_packet()
        except Exception:
            self.get_logger().error(
                f"Serial read from the Teensy failed, skipping this telemetry cycle\n{traceback.format_exc()}"
            )
            return

        if message_type != RxMessageType.DATA:
            return

        sample = payload[-1]
        if len(sample) != len(TELEMETRY_FIELDS):
            return
        telemetry = dict(zip(TELEMETRY_FIELDS, sample))

        self.emergency_stop = bool(telemetry["stop_button"])

        message = Teensy()
        message.emergency_stop = bool(telemetry["stop_button"])
        message.right_angle = float(telemetry["motor_right_angle"])
        message.left_angle = float(telemetry["motor_left_angle"])
        message.right_velocity = float(telemetry["motor_right_speed"])
        message.left_velocity = float(telemetry["motor_left_speed"])
        message.right_torque = float(telemetry["motor_right_torque"])
        message.left_torque = float(telemetry["motor_left_torque"])
        message.rotation_speed = float(telemetry["cybergear_rotation_speed"])
        message.azimuth_angle = float(telemetry["cybergear_azimuth_angle"])
        message.force = float(telemetry["handle_force_x"])
        self.telemetry_publisher.publish(message)

    def destroy_node(self):
        try:
            self.teensy.set_drive_speed(0.0, 0.0)
            self.teensy.free_handle_motor()
        except Exception:
            self.get_logger().error(f"Could not stop the motors on shutdown\n{traceback.format_exc()}")
        super().destroy_node()


def main():
    rclpy.init()
    node = TeensyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
