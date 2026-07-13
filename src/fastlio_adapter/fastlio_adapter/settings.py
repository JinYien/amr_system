from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class TopicSettings:
    odometry_subscriber: str
    odometry_publisher: str


@dataclass(frozen=True)
class FrameSettings:
    odom: str
    base: str
    sensor: str


@dataclass(frozen=True)
class OptionSettings:
    zero_first_pose: bool
    publish_tf: bool


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    frames: FrameSettings
    options: OptionSettings


def load_settings(path: str) -> Settings:
    with open(path, "r") as handle:
        root = yaml.safe_load(handle)
    return Settings(
        topics=TopicSettings(**root["topics"]),
        frames=FrameSettings(**root["frames"]),
        options=OptionSettings(**root["options"]),
    )
