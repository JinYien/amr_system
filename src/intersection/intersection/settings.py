from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class TopicSettings:
    scan_subscriber: str
    control_subscriber: str
    intersection_publisher: str
    marker_publisher: str
    sound_publisher: str


@dataclass(frozen=True)
class SoundSettings:
    enable: bool
    clips: dict


@dataclass(frozen=True)
class RobotSettings:
    width: float
    safety_margin: float


@dataclass(frozen=True)
class DetectionSettings:
    lidar_yaw_offset: float
    min_open_distance: float
    open_space_distance: float
    open_space_hysteresis: float
    front_min: float
    front_max: float
    left_min: float
    left_max: float
    right_min: float
    right_max: float
    debounce_time: float
    detection_rate: float


@dataclass(frozen=True)
class VisualizationSettings:
    enable: bool
    marker_lifetime: float
    sector_alpha: float
    point_size: float


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    sounds: SoundSettings
    robot: RobotSettings
    detection: DetectionSettings
    visualization: VisualizationSettings


def load_settings(path: str) -> Settings:
    with open(path, "r") as handle:
        root = yaml.safe_load(handle)

    sounds = root["sounds"]
    return Settings(
        topics=TopicSettings(**root["topics"]),
        sounds=SoundSettings(
            enable=sounds["enable"],
            clips={key: value for key, value in sounds.items() if key != "enable"},
        ),
        robot=RobotSettings(**root["robot"]),
        detection=DetectionSettings(**root["detection"]),
        visualization=VisualizationSettings(**root["visualization"]),
    )
