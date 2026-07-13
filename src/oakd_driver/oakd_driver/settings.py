import os
from dataclasses import dataclass
import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class TopicSettings:
    detection_publisher: str
    image_publisher: str
    sound_publisher: str
    control_subscriber: str


@dataclass(frozen=True)
class SoundSettings:
    enable: bool
    forget_time: float


@dataclass(frozen=True)
class ModelSettings:
    blob_path: str
    config_path: str
    shaves: int
    confidence_threshold: float


@dataclass(frozen=True)
class DeviceSettings:
    fps: float
    resolution: str
    keep_aspect_ratio: bool
    reconnect_delay: float


@dataclass(frozen=True)
class VisualizationSettings:
    enable: bool
    publish_rate: float
    jpeg_quality: int


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    sounds: SoundSettings
    device: DeviceSettings
    model: ModelSettings
    visualization: VisualizationSettings


def resolve_path(path: str) -> str:
    path = os.path.expanduser(path)
    if os.path.isabs(path):
        return path
    return os.path.join(get_package_share_directory("oakd_driver"), path)


def load_settings(config_path: str) -> Settings:
    with open(config_path, "r") as handle:
        config = yaml.safe_load(handle)

    return Settings(
        topics=TopicSettings(
            detection_publisher=config["topics"]["detection_publisher"],
            image_publisher=config["topics"]["image_publisher"],
            sound_publisher=config["topics"]["sound_publisher"],
            control_subscriber=config["topics"]["control_subscriber"],
        ),
        sounds=SoundSettings(
            enable=bool(config["sounds"]["enable"]),
            forget_time=float(config["sounds"]["forget_time"]),
        ),
        model=ModelSettings(
            blob_path=resolve_path(config["model"]["blob_path"]),
            config_path=resolve_path(config["model"]["config_path"]),
            shaves=int(config["model"]["shaves"]),
            confidence_threshold=float(config["model"]["confidence_threshold"]),
        ),
        device=DeviceSettings(
            fps=float(config["device"]["fps"]),
            resolution=str(config["device"]["resolution"]),
            keep_aspect_ratio=bool(config["device"]["keep_aspect_ratio"]),
            reconnect_delay=float(config["device"]["reconnect_delay"]),
        ),
        visualization=VisualizationSettings(
            enable=bool(config["visualization"]["enable"]),
            publish_rate=float(config["visualization"]["publish_rate"]),
            jpeg_quality=int(config["visualization"]["jpeg_quality"]),
        ),
    )
