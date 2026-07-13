from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class TopicSettings:
    sound_subscriber: str


@dataclass(frozen=True)
class BeepSettings:
    near_distance: float
    far_distance: float
    near_interval: float
    far_interval: float
    gap: float
    frequency: float
    volume: float


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    beep: BeepSettings


def load_settings(path: str) -> Settings:
    with open(path, "r") as handle:
        root = yaml.safe_load(handle)
    return Settings(
        topics=TopicSettings(**root["topics"]),
        beep=BeepSettings(**root["beep"]),
    )
