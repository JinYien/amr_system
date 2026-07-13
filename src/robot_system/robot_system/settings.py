from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class TopicSettings:
    drive_command_publisher: str
    handle_command_publisher: str
    control_subscriber: str
    joystick_subscriber: str
    telemetry_subscriber: str
    velocity_command_subscriber: str
    user_velocity_publisher: str
    scan_subscriber: str
    mix_control_publisher: str
    sound_publisher: str


@dataclass(frozen=True)
class WheelSettings:
    radius: float
    track: float


@dataclass(frozen=True)
class FootprintSettings:
    frame: str
    length: float
    width: float


@dataclass(frozen=True)
class ControlLoopSettings:
    period: float
    command_timeout: float
    scan_timeout: float


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
    gain: float
    max_acceleration: float
    deadzone: float


@dataclass(frozen=True)
class MixSettings:
    obstacle_cone: float
    steer_cone: float
    user_full_distance: float
    robot_full_distance: float
    blind_latch_distance: float
    max_forward_velocity: float
    max_reverse_velocity: float
    max_angular_velocity: float
    stop_hold_time: float
    safety_cap: bool


@dataclass(frozen=True)
class NudgeSettings:
    align_angle: float
    repeat_interval: float


@dataclass(frozen=True)
class PulseSettings:
    distance: float
    repeat_interval: float


@dataclass(frozen=True)
class Settings:
    topics: TopicSettings
    wheel: WheelSettings
    footprint: FootprintSettings
    control_loop: ControlLoopSettings
    force: ForceSettings
    linear: LinearSettings
    angular: AngularSettings
    mix: MixSettings
    nudge: NudgeSettings
    pulse: PulseSettings


def load_settings(path: str) -> Settings:
    with open(path, "r") as handle:
        root = yaml.safe_load(handle)

    return Settings(
        topics=TopicSettings(**root["topics"]),
        wheel=WheelSettings(**root["wheel"]),
        footprint=FootprintSettings(**root["footprint"]),
        control_loop=ControlLoopSettings(**root["control_loop"]),
        force=ForceSettings(**root["force"]),
        linear=LinearSettings(**root["linear"]),
        angular=AngularSettings(**root["angular"]),
        mix=MixSettings(**root["mix"]),
        nudge=NudgeSettings(**root["nudge"]),
        pulse=PulseSettings(**root["pulse"]),
    )
