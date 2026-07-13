import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from custom_message.msg import JoystickCommand, Control, DriveCommand, HandleCommand, Teensy, MixControl, Sound
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_system.force import ForceCommand
from robot_system.settings import Settings, load_settings
from robot_system.differential_drive import forward_kinematics
from robot_system.mix import MixController, scan_front_distance, scan_bearing_distance

BEEP_CLEAR_DISTANCE = 1000.0
MIN_CLEARANCE = 0.01


class RobotSystemNode(Node):
    def __init__(self):
        super().__init__("robot_system_node")

        # ======================================================
        self.get_logger().info("Initializing node ...")
        # ======================================================
        config_path = self.declare_parameter("config_path", "").get_parameter_value().string_value
        self.settings: Settings = load_settings(config_path)

        self.mode = "manual"
        self.authority = ""
        self.force = 0.0
        self.azimuth_angle = 0.0
        self.velocity_command = None
        self.velocity_command_time = None
        self.front_distance = float("inf")
        self.scan = None
        self.scan_time = None
        self.last_nudge_time = None
        self.last_pulse_time = 0.0

        self.wheel = self.settings.wheel
        self.force_command = ForceCommand(
            self.settings.control_loop.period, self.settings.force, self.settings.linear, self.settings.angular, self.wheel
        )
        self.mix_controller = MixController(self.settings.mix, self.wheel)
        self.obstacle_cone = np.deg2rad(self.settings.mix.obstacle_cone)
        self.steer_cone = np.deg2rad(self.settings.mix.steer_cone)
        self.front_offset = self.settings.footprint.length / 2.0
        self.nudge_align = np.deg2rad(self.settings.nudge.align_angle)
        self.user_stop = False
        self.pull_start_time = None
        self.steer_clearance = float("inf")

        # ======================================================
        self.get_logger().info("Publishing & subscribing topics ...")
        # ======================================================
        topics = self.settings.topics
        qos = qos_profile_sensor_data

        self.drive_command_publisher = self.create_publisher(DriveCommand, topics.drive_command_publisher, qos)
        self.handle_command_publisher = self.create_publisher(HandleCommand, topics.handle_command_publisher, qos)
        self.user_velocity_publisher = self.create_publisher(Twist, topics.user_velocity_publisher, 10)
        self.mix_control_publisher = self.create_publisher(MixControl, topics.mix_control_publisher, qos)
        self.sound_publisher = self.create_publisher(Sound, topics.sound_publisher, qos)

        self.create_subscription(Control, topics.control_subscriber, self.control_callback, qos)
        self.create_subscription(JoystickCommand, topics.joystick_subscriber, self.joystick_callback, qos)
        self.create_subscription(Teensy, topics.telemetry_subscriber, self.telemetry_callback, qos)
        self.create_subscription(Twist, topics.velocity_command_subscriber, self.velocity_command_callback, 10)
        self.create_subscription(LaserScan, topics.scan_subscriber, self.scan_callback, qos)

        # ======================================================
        self.get_logger().info("Running process ...")
        # ======================================================
        self.create_timer(self.settings.control_loop.period, self.control_loop)

    def control_callback(self, message: Control):
        entering_mix = (message.mode == "auto" and message.authority == "mix") and not (
            self.mode == "auto" and self.authority == "mix"
        )
        self.mode = message.mode
        self.authority = message.authority
        if entering_mix:
            self.mix_controller.reset()
            self.user_stop = False
            self.pull_start_time = None

    def joystick_callback(self, message: JoystickCommand):
        if self.mode != "manual":
            return
        left, right = forward_kinematics(message.linear_velocity, message.angular_velocity, self.wheel)
        self.publish_drive(np.rad2deg(left), np.rad2deg(right))
        if message.middle_mode:
            handle = HandleCommand()
            handle.mode = message.middle_mode
            handle.action = message.middle_action
            self.handle_command_publisher.publish(handle)

    def telemetry_callback(self, message: Teensy):
        self.force = float(message.force)
        self.azimuth_angle = float(message.azimuth_angle)

    def velocity_command_callback(self, message: Twist):
        self.velocity_command = (float(message.linear.x), float(message.angular.z))
        self.velocity_command_time = self.now()

    def scan_callback(self, message: LaserScan):
        self.scan = message
        self.scan_time = self.now()
        raw = scan_front_distance(message, self.obstacle_cone)
        clearance = max(MIN_CLEARANCE, raw - self.front_offset)
        no_return = raw >= float(message.range_max)
        if no_return and self.front_distance <= self.settings.mix.blind_latch_distance:
            clearance = self.front_distance
        self.front_distance = clearance

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def velocity_command_fresh(self) -> bool:
        if self.velocity_command_time is None:
            return False
        return self.now() - self.velocity_command_time <= self.settings.control_loop.command_timeout

    def scan_fresh(self) -> bool:
        if self.scan_time is None:
            return False
        return self.now() - self.scan_time <= self.settings.control_loop.scan_timeout

    def control_loop(self):
        scan_ok = self.scan_fresh()
        self.steer_clearance = self.update_steer_clearance() if scan_ok else MIN_CLEARANCE
        self.apply_nudge()
        self.apply_pulse()

        user_wheels = self.force_command.update(self.force, self.azimuth_angle)
        self.publish_user_velocity(self.force_command.linear_velocity, self.force_command.angular_velocity)
        robot_twist = self.velocity_command if self.velocity_command_fresh() else (0.0, 0.0)
        front_distance = self.front_distance if scan_ok else MIN_CLEARANCE
        steer_distance = self.steer_clearance
        beep_active = scan_ok and self.mode == "auto" and self.authority in ("robot", "mix")
        beep_distance = front_distance if beep_active else BEEP_CLEAR_DISTANCE
        self.sound_publisher.publish(Sound(obstacle_distance=float(beep_distance)))
        in_mix = self.mode == "auto" and self.authority == "mix"
        if in_mix:
            self.update_stop_latch()
            if self.user_stop:
                robot_twist = (0.0, 0.0)
        output = self.mix_controller.update(
            user_wheels,
            robot_twist,
            front_distance,
            steer_distance,
            self.is_intervening(),
            self.user_stop,
            self.settings.control_loop.period,
            in_mix,
        )
        self.publish_mix_control(output)

        if self.mode == "auto" and self.authority == "user":
            self.publish_drive(user_wheels[0], user_wheels[1])
        elif self.mode == "auto" and self.authority == "robot":
            if self.velocity_command is not None and self.velocity_command_fresh():
                left, right = forward_kinematics(robot_twist[0], robot_twist[1], self.wheel)
                self.publish_drive(np.rad2deg(left), np.rad2deg(right))
        elif in_mix:
            self.publish_drive(output.left_velocity, output.right_velocity)

    def is_intervening(self) -> bool:
        return (
            abs(self.force_command.force_filtered) > self.settings.force.deadzone
            or abs(self.azimuth_angle) > self.settings.angular.deadzone
        )

    def update_steer_clearance(self) -> float:
        if self.scan is None:
            return MIN_CLEARANCE
        raw = scan_bearing_distance(self.scan, -self.azimuth_angle, self.steer_cone)
        clearance = max(MIN_CLEARANCE, raw - self.front_offset)
        no_return = raw >= float(self.scan.range_max)
        if no_return and self.steer_clearance <= self.settings.mix.blind_latch_distance:
            clearance = self.steer_clearance
        return clearance

    def update_stop_latch(self):
        force = self.force_command.force_filtered
        deadzone = self.settings.force.deadzone
        if force < -deadzone:
            if self.pull_start_time is None:
                self.pull_start_time = self.now()
            elif self.now() - self.pull_start_time >= self.settings.mix.stop_hold_time:
                self.user_stop = True
        else:
            self.pull_start_time = None
            if force > deadzone:
                self.user_stop = False

    def apply_nudge(self):
        if not (self.mode == "auto" and self.authority in ("robot", "mix")):
            return
        if abs(self.azimuth_angle) <= self.nudge_align:
            self.last_nudge_time = None
            return
        now = self.now()
        if self.last_nudge_time is not None and now - self.last_nudge_time < self.settings.nudge.repeat_interval:
            return
        self.last_nudge_time = now
        self.publish_nudge("left" if self.azimuth_angle > 0.0 else "right")

    def apply_pulse(self):
        if not (self.mode == "auto" and self.authority == "mix"):
            return
        if not self.scan_fresh():
            return
        if self.steer_clearance > self.settings.pulse.distance:
            return
        now = self.now()
        if now - self.last_pulse_time < self.settings.pulse.repeat_interval:
            return
        self.last_pulse_time = now
        self.handle_command_publisher.publish(HandleCommand(mode="pulse"))

    def publish_nudge(self, direction):
        message = HandleCommand()
        message.mode = "nudge"
        message.action = direction
        self.handle_command_publisher.publish(message)

    def publish_drive(self, left_velocity, right_velocity):
        message = DriveCommand()
        message.left_velocity = float(left_velocity)
        message.right_velocity = float(right_velocity)
        self.drive_command_publisher.publish(message)

    def publish_user_velocity(self, linear_velocity, angular_velocity):
        message = Twist()
        message.linear.x = float(linear_velocity)
        message.angular.z = float(angular_velocity)
        self.user_velocity_publisher.publish(message)

    def publish_mix_control(self, output):
        message = MixControl()
        message.front_distance = float(output.front_distance)
        message.steer_distance = float(output.steer_distance)
        message.user_weight = float(output.user_weight)
        message.mix_linear = float(output.mix_linear)
        message.mix_angular = float(output.mix_angular)
        message.intervening = bool(output.intervening)
        message.user_control = bool(output.user_control)
        message.stopped = bool(output.stopped)
        message.user_time = float(output.user_time)
        message.robot_time = float(output.robot_time)
        message.stop_time = float(output.stop_time)
        self.mix_control_publisher.publish(message)


def main():
    rclpy.init()
    node = RobotSystemNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
