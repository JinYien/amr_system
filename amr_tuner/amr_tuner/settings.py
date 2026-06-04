#!/usr/bin/env python3

from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class TopicSettings:
    gain: str
    command: str
    teensy: str


@dataclass(frozen=True)
class RateSettings:
    serial_read: int
    drive_command: int
    ros_spin_interval: int


@dataclass(frozen=True)
class SerialSettings:
    identifier: str
    baudrate: int
    timeout: float
    boot_delay: float
    gain_apply_delay: float


@dataclass(frozen=True)
class WindowSettings:
    width: int
    height: int
    x: int
    y: int
    left_panel: int
    right_panel: int
    gui_update_interval: int


@dataclass(frozen=True)
class RangeSettings:
    min: float
    max: float
    step: float
    decimals: int


@dataclass(frozen=True)
class VelocitySettings:
    min: float
    max: float
    slider_scale: int
    slider_tick_interval: int
    step: float
    decimals: int


@dataclass(frozen=True)
class WheelSettings:
    min_radius: float
    max_radius: float
    radius_step: float
    radius_decimals: int
    default_radius: float


@dataclass(frozen=True)
class GraphSettings:
    decimal_precision: int
    min_time_window: float
    max_time_window: float
    time_window_step: float
    time_window_decimals: int
    default_time_window: float
    amplitude_sample_size: int


@dataclass(frozen=True)
class AnalysisSettings:
    min_valid_period: float
    max_valid_period: float
    peak_detection_window: int
    min_oscillation_periods: int
    average_period_window: int


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    rates: RateSettings
    serial: SerialSettings
    window: WindowSettings
    pid_gain: RangeSettings
    velocity: VelocitySettings
    wheel: WheelSettings
    linear_velocity: RangeSettings
    graph: GraphSettings
    analysis: AnalysisSettings


def load_settings(path: str) -> Settings:
    """tuner.yaml を読み込み Settings を生成する。"""
    with open(path, "r") as handle:
        raw = yaml.safe_load(handle)

    root = raw["tuner"]
    return Settings(
        topics=TopicSettings(**root["topics"]),
        rates=RateSettings(**root["rates"]),
        serial=SerialSettings(**root["serial"]),
        window=WindowSettings(**root["window"]),
        pid_gain=RangeSettings(**root["pid_gain"]),
        velocity=VelocitySettings(**root["velocity"]),
        wheel=WheelSettings(**root["wheel"]),
        linear_velocity=RangeSettings(**root["linear_velocity"]),
        graph=GraphSettings(**root["graph"]),
        analysis=AnalysisSettings(**root["analysis"]),
    )
