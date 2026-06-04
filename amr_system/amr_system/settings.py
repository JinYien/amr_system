#!/usr/bin/env python3

from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class LoopSettings:
    period: float


@dataclass(frozen=True)
class WheelSettings:
    radius: float
    track: float


@dataclass(frozen=True)
class ForceSettings:
    deadzone: float
    clamp_min: float
    clamp_max: float
    useful_range: float
    low_pass_alpha: float


@dataclass(frozen=True)
class LinearSettings:
    forward_gain: float
    reverse_gain: float
    max: float
    min: float
    max_acceleration: float
    max_deceleration: float


@dataclass(frozen=True)
class AngularSettings:
    max_acceleration: float


@dataclass(frozen=True)
class TopicSettings:
    command_publisher: str
    control_subscriber: str
    joystick_subscriber: str
    robot_subscriber: str
    teensy_subscriber: str


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    loop: LoopSettings
    wheel: WheelSettings
    force: ForceSettings
    linear: LinearSettings
    angular: AngularSettings


def load_settings(path: str, robot_path: str) -> Settings:
    with open(path, "r") as handle:
        raw = yaml.safe_load(handle)
    with open(robot_path, "r") as handle:
        robot = yaml.safe_load(handle)["amr_robot"]

    root = raw["command"]
    return Settings(
        topics=TopicSettings(**root["topics"]),
        loop=LoopSettings(**root["loop"]),
        wheel=WheelSettings(**robot["wheel"]),
        force=ForceSettings(**root["force"]),
        linear=LinearSettings(**root["linear"]),
        angular=AngularSettings(**root["angular"]),
    )
