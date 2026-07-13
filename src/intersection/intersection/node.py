import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from custom_message.msg import Control, Intersection, Sound
from intersection.settings import load_settings
from intersection.geometry import Sector
from intersection.detector import Detector, DetectorParams, Decision, classify_junction, JUNCTION_LABELS, UNKNOWN
from intersection.debounce import DebounceFilter
from intersection.visualization import Visualizer


class IntersectionNode(Node):
    def __init__(self):
        super().__init__("intersection_node")

        # ======================================================
        self.get_logger().info("Initializing node ...")
        # ======================================================
        config_path = self.declare_parameter("config_path", "").get_parameter_value().string_value
        self.settings = load_settings(config_path)
        detection = self.settings.detection

        required_clearance = self.settings.robot.width + 2.0 * self.settings.robot.safety_margin
        # the config thresholds are measured from the footprint edge, the scan
        # ranges from the robot centre, so the half body is added here
        half_body = self.settings.robot.width / 2.0
        self.detector_params = DetectorParams(
            front=Sector(math.radians(detection.front_min), math.radians(detection.front_max)),
            left=Sector(math.radians(detection.left_min), math.radians(detection.left_max)),
            right=Sector(math.radians(detection.right_min), math.radians(detection.right_max)),
            min_open_distance=detection.min_open_distance + half_body,
            required_clearance=required_clearance,
            lidar_yaw_offset=math.radians(detection.lidar_yaw_offset),
            open_space_distance=detection.open_space_distance + half_body,
        )
        self.detector = Detector(self.detector_params)
        self.front_filter = DebounceFilter(detection.debounce_time)
        self.left_filter = DebounceFilter(detection.debounce_time)
        self.right_filter = DebounceFilter(detection.debounce_time)
        self.decision = Decision()
        self.detection = None
        self.detection_period = 1.0 / detection.detection_rate if detection.detection_rate > 0.0 else 0.0
        self.last_detection_time = 0.0

        self.sound_clips = {
            junction: self.settings.sounds.clips.get(label)
            for junction, label in JUNCTION_LABELS.items()
            if self.settings.sounds.clips.get(label)
        }
        self.last_sound_junction = UNKNOWN
        self.last_junction = UNKNOWN
        self.mode = "manual"
        self.authority = ""

        # ======================================================
        self.get_logger().info("Publishing & subscribing topics ...")
        # ======================================================
        topics = self.settings.topics
        self.intersection_publisher = self.create_publisher(Intersection, topics.intersection_publisher, qos_profile_sensor_data)
        self.sound_publisher = None
        if self.settings.sounds.enable:
            self.sound_publisher = self.create_publisher(Sound, topics.sound_publisher, qos_profile_sensor_data)
        self.create_subscription(LaserScan, topics.scan_subscriber, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Control, topics.control_subscriber, self.control_callback, qos_profile_sensor_data)
        self.visualizer = None
        if self.settings.visualization.enable:
            self.visualizer = Visualizer(self, topics.marker_publisher, self.settings.visualization)

        # ======================================================
        self.get_logger().info("Running process ...")
        # ======================================================

    def scan_callback(self, scan: LaserScan):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.detection_period <= 0.0 or (now - self.last_detection_time) >= self.detection_period:
            self.last_detection_time = now
            self.detection = self.detector.detect(scan)
            self.decision.front = self.front_filter.update(self.detection.front.open, now)
            self.decision.left = self.left_filter.update(self.detection.left.open, now)
            self.decision.right = self.right_filter.update(self.detection.right.open, now)

        if self.detection is None:
            return

        junction = classify_junction(
            self.decision.front,
            self.decision.left,
            self.decision.right,
            self.detection.nearest,
            self.detector_params.open_space_distance,
            self.last_junction,
            self.settings.detection.open_space_hysteresis,
        )
        self.last_junction = junction
        self.publish_intersection(scan, junction)
        self.play_junction_sound(junction)

        if self.visualizer is not None and self.visualizer.has_subscribers():
            self.visualizer.publish(scan, self.detector_params, self.detection, self.decision, junction)

    def publish_intersection(self, scan: LaserScan, junction: int):
        message = Intersection()
        message.header = scan.header
        message.junction = junction
        message.front_open = self.decision.front
        message.left_open = self.decision.left
        message.right_open = self.decision.right
        self.intersection_publisher.publish(message)

    def control_callback(self, message: Control):
        self.mode = message.mode
        self.authority = message.authority

    def play_junction_sound(self, junction: int):
        if self.sound_publisher is None or junction == self.last_sound_junction:
            return
        first = self.last_sound_junction == UNKNOWN
        self.last_sound_junction = junction
        if first:
            return
        if self.mode == "auto" and self.authority == "user":
            return
        clip = self.sound_clips.get(junction)
        if clip:
            self.sound_publisher.publish(Sound(object=clip))


def main():
    rclpy.init()
    node = IntersectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
