#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import Tuple
import yaml


@dataclass(frozen=True)
class TopicSettings:
    teensy_publisher: str
    state_publisher: str
    odometry_publisher: str
    command_subscriber: str


@dataclass(frozen=True)
class FrameSettings:
    odometry: str
    base: str


@dataclass(frozen=True)
class RateSettings:
    serial_read: int
    drive_command: int
    handle_command: int


@dataclass(frozen=True)
class SerialSettings:
    identifier: str
    baudrate: int
    timeout: float
    boot_delay: float
    motor_setup_delay: float


@dataclass(frozen=True)
class WheelSettings:
    radius: float
    track: float


@dataclass(frozen=True)
class PidGains:
    p_gain: float
    i_gain: float
    d_gain: float


@dataclass(frozen=True)
class DriveSettings:
    left: PidGains
    right: PidGains


@dataclass(frozen=True)
class HapticPattern:
    torque: float
    hold_time: float
    sequence: Tuple[float, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class HandleSettings:
    torque_limit: float
    pulse: HapticPattern
    nudge: HapticPattern


@dataclass(frozen=True)
class OdometrySettings:
    max_timestep: float


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    frames: FrameSettings
    rates: RateSettings
    serial: SerialSettings
    wheel: WheelSettings
    drive: DriveSettings
    handle: HandleSettings
    odometry: OdometrySettings


def haptic_pattern(raw: dict) -> HapticPattern:
    return HapticPattern(
        torque=raw["torque"],
        hold_time=raw["hold_time"],
        sequence=tuple(raw.get("sequence", ())),
    )


def load_settings(path: str, robot_path: str) -> Settings:
    with open(path, "r") as handle:
        raw = yaml.safe_load(handle)
    with open(robot_path, "r") as handle:
        robot = yaml.safe_load(handle)["amr_robot"]

    root = raw["teensy"]

    return Settings(
        topics=TopicSettings(**root["topics"]),
        frames=FrameSettings(**root["frames"]),
        rates=RateSettings(**root["rates"]),
        serial=SerialSettings(**root["serial"]),
        wheel=WheelSettings(**robot["wheel"]),
        drive=DriveSettings(
            left=PidGains(**root["drive"]["left"]),
            right=PidGains(**root["drive"]["right"]),
        ),
        handle=HandleSettings(
            torque_limit=root["handle"]["torque_limit"],
            pulse=haptic_pattern(root["handle"]["pulse"]),
            nudge=haptic_pattern(root["handle"]["nudge"]),
        ),
        odometry=OdometrySettings(**root["odometry"]),
    )
