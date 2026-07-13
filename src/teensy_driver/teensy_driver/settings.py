from dataclasses import dataclass, field
from typing import Tuple
import yaml


@dataclass(frozen=True)
class TopicSettings:
    telemetry_publisher: str
    drive_command_subscriber: str
    handle_command_subscriber: str


@dataclass(frozen=True)
class RateSettings:
    serial_read: int
    drive_command: int
    handle_command: int


@dataclass(frozen=True)
class SafetySettings:
    command_timeout: float


@dataclass(frozen=True)
class SerialSettings:
    usb_id: str
    baudrate: int
    read_timeout: float
    boot_delay: float
    motor_setup_delay: float


@dataclass(frozen=True)
class HapticPattern:
    torque: float
    hold_time: float
    sequence: Tuple[float, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class MiddleSettings:
    torque_limit: float
    pulse: HapticPattern
    nudge: HapticPattern


@dataclass(frozen=True)
class PidGains:
    p_gain: float
    i_gain: float
    d_gain: float


@dataclass(frozen=True)
class MotorSettings:
    left: PidGains
    right: PidGains
    middle: MiddleSettings


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    serial: SerialSettings
    rates: RateSettings
    safety: SafetySettings
    motor: MotorSettings


def haptic_pattern(raw: dict) -> HapticPattern:
    return HapticPattern(
        torque=raw["torque"],
        hold_time=raw["hold_time"],
        sequence=tuple(raw.get("sequence", ())),
    )


def load_settings(path: str) -> Settings:
    with open(path, "r") as handle:
        root = yaml.safe_load(handle)

    motor = root["motor"]
    return Settings(
        topics=TopicSettings(**root["topics"]),
        serial=SerialSettings(**root["serial"]),
        rates=RateSettings(**root["rates"]),
        safety=SafetySettings(**root["safety"]),
        motor=MotorSettings(
            left=PidGains(**motor["left"]),
            right=PidGains(**motor["right"]),
            middle=MiddleSettings(
                torque_limit=motor["middle"]["torque_limit"],
                pulse=haptic_pattern(motor["middle"]["pulse"]),
                nudge=haptic_pattern(motor["middle"]["nudge"]),
            ),
        ),
    )
