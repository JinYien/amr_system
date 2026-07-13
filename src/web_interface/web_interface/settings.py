from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class TopicSettings:
    joystick_publisher: str
    control_publisher: str
    sound_publisher: str
    initial_pose_publisher: str
    navigate_action: str
    navigate_through_action: str
    robot_pose_subscriber: str


@dataclass(frozen=True)
class SoundSettings:
    goal_reached: str


@dataclass(frozen=True)
class ServerSettings:
    host: str
    port: int


@dataclass(frozen=True)
class LoggingSettings:
    port: int
    rate: float
    path: str


@dataclass(frozen=True)
class MotionSettings:
    publish_rate: int
    max_linear_velocity: float
    max_angular_velocity: float


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    sounds: SoundSettings
    server: ServerSettings
    logging: LoggingSettings
    motion: MotionSettings


def load_settings(path: str) -> Settings:
    with open(path, "r") as handle:
        root = yaml.safe_load(handle)
    return Settings(
        topics=TopicSettings(**root["topics"]),
        sounds=SoundSettings(**root["sounds"]),
        server=ServerSettings(**root["server"]),
        logging=LoggingSettings(**root["logging"]),
        motion=MotionSettings(**root["motion"]),
    )
