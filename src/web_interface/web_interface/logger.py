import csv
import json
import math
import threading
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from custom_message.msg import (
    Control,
    DetectionArray,
    DriveCommand,
    HandleCommand,
    Intersection,
    MixControl,
    Sound,
    Teensy,
)

LOG_TOPICS = (
    ("telemetry", "/teensy/telemetry", Teensy),
    ("control", "/interface/control", Control),
    ("drive", "/command/drive", DriveCommand),
    ("handle", "/command/handle", HandleCommand),
    ("user", "/command/user", Twist),
    ("robot", "/command/robot", Twist),
    ("mix", "/command/mix", MixControl),
    ("sound", "/command/sound", Sound),
    ("detection", "/oakd/detection", DetectionArray),
    ("intersection", "/intersection/state", Intersection),
    ("pose", "/amcl_pose", PoseWithCovarianceStamped),
)

SKIP_FIELDS = ("header", "covariance")
DISTANCE_JUMP_GUARD = 0.5
OFFSET_FIELDS = {"pose": ("x", "y", "yaw"), "mix": ("user_time", "robot_time", "stop_time")}


def pose_values(message) -> dict:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return {"x": position.x, "y": position.y, "yaw": yaw}


CONVERTERS = {"pose": pose_values}


def flatten_message(message, prefix="", result=None):
    if result is None:
        result = {}
    for field in message.get_fields_and_field_types():
        if field in SKIP_FIELDS:
            continue
        value = getattr(message, field)
        name = prefix + field
        if hasattr(value, "get_fields_and_field_types"):
            flatten_message(value, name + "_", result)
        elif isinstance(value, (list, tuple)):
            items = []
            for item in value:
                if hasattr(item, "get_fields_and_field_types"):
                    items.append(flatten_message(item, "", {}))
                else:
                    items.append(item)
            result[name] = json.dumps(items)
        elif isinstance(value, float):
            result[name] = value if math.isfinite(value) else None
        else:
            result[name] = value
    return result


class TopicLogger:
    def __init__(self, node, settings):
        self.node = node
        self.settings = settings
        self.directory = Path(get_package_share_directory("web_interface")) / settings.path
        self.directory.mkdir(parents=True, exist_ok=True)

        self.lock = threading.Lock()
        self.latest = {}
        self.topic_columns = {}
        for key, _, message_type in LOG_TOPICS:
            convert = CONVERTERS.get(key, flatten_message)
            self.latest[key] = convert(message_type())
            self.topic_columns[key] = list(self.latest[key])
        self.columns = ["time", "distance"]
        for key, _, _ in LOG_TOPICS:
            self.columns += [f"{key}_{name}" for name in self.topic_columns[key]]

        self.recording = False
        self.start_time = 0.0
        self.distance = 0.0
        self.last_position = None
        self.baseline = {}
        self.file = None
        self.writer = None
        self.current_file = None
        self.last_file = None

        for key, topic, message_type in LOG_TOPICS:
            node.create_subscription(message_type, topic, self.make_callback(key), qos_profile_sensor_data)
        node.create_timer(1.0 / settings.rate, self.tick)

    def make_callback(self, key):
        convert = CONVERTERS.get(key, flatten_message)

        def callback(message):
            with self.lock:
                self.latest[key] = convert(message)
        return callback

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def start(self):
        with self.lock:
            if self.recording:
                return
            name = time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
            self.current_file = name
            self.file = (self.directory / name).open("w", newline="")
            self.writer = csv.writer(self.file)
            self.writer.writerow(self.columns)
            self.start_time = self.now()
            self.distance = 0.0
            self.last_position = None
            self.baseline = {
                key: {name: float(self.latest[key].get(name) or 0.0) for name in fields}
                for key, fields in OFFSET_FIELDS.items()
            }
            self.recording = True

    def stop(self) -> str:
        with self.lock:
            if not self.recording:
                return self.last_file or ""
            self.recording = False
            self.file.close()
            self.last_file = self.current_file
            self.file = None
            self.writer = None
            self.current_file = None
            return self.last_file

    def offset_value(self, key, name, value):
        fields = OFFSET_FIELDS.get(key)
        if fields is None or name not in fields or not isinstance(value, (int, float)):
            return value
        base = self.baseline[key][name]
        if name == "yaw":
            return math.atan2(math.sin(value - base), math.cos(value - base))
        if key == "mix" and value < base:
            self.baseline[key][name] = 0.0
            return value
        return value - base

    def update_distance(self):
        position = (self.latest["pose"]["x"], self.latest["pose"]["y"])
        if self.last_position is not None:
            step = math.hypot(position[0] - self.last_position[0], position[1] - self.last_position[1])
            if step < DISTANCE_JUMP_GUARD:
                self.distance += step
        self.last_position = position

    def tick(self):
        with self.lock:
            if not self.recording:
                return
            self.update_distance()
            row = [round(self.now() - self.start_time, 3), round(self.distance, 3)]
            for key, _, _ in LOG_TOPICS:
                values = self.latest[key]
                row += [self.offset_value(key, name, values.get(name)) for name in self.topic_columns[key]]
            self.writer.writerow(row)
            self.file.flush()

    def file_path(self, name: str):
        candidate = self.directory / Path(name).name
        return candidate if candidate.is_file() else None

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "recording": self.recording,
                "elapsed": round(self.now() - self.start_time, 1) if self.recording else 0.0,
                "distance": round(self.distance, 2) if self.recording else 0.0,
                "topics": {topic: dict(self.latest[key]) for key, topic, _ in LOG_TOPICS},
            }
