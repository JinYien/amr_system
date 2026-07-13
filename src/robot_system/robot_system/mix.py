from dataclasses import dataclass
import numpy as np
from robot_system.differential_drive import forward_kinematics, inverse_kinematics


@dataclass
class MixOutput:
    left_velocity: float
    right_velocity: float
    front_distance: float
    steer_distance: float
    user_weight: float
    mix_linear: float
    mix_angular: float
    intervening: bool
    user_control: bool
    stopped: bool
    user_time: float
    robot_time: float
    stop_time: float


def scan_front_distance(scan, cone_angle: float) -> float:
    return scan_bearing_distance(scan, 0.0, cone_angle)


def scan_bearing_distance(scan, bearing: float, window: float) -> float:
    ranges = np.asarray(scan.ranges, dtype=float)
    if ranges.size == 0:
        return float(scan.range_max)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    mask = (
        (angles >= bearing - window)
        & (angles <= bearing + window)
        & np.isfinite(ranges)
        & (ranges >= scan.range_min)
        & (ranges <= scan.range_max)
    )
    if not np.any(mask):
        return float(scan.range_max)
    return float(np.min(ranges[mask]))


class MixController:
    def __init__(self, settings, wheel):
        self.settings = settings
        self.wheel = wheel
        self.user_time = 0.0
        self.robot_time = 0.0
        self.stop_time = 0.0

    def reset(self):
        self.user_time = 0.0
        self.robot_time = 0.0
        self.stop_time = 0.0

    def user_weight(self, front_distance: float) -> float:
        span = self.settings.user_full_distance - self.settings.robot_full_distance
        if span <= 0.0:
            return 1.0
        return float(np.clip((front_distance - self.settings.robot_full_distance) / span, 0.0, 1.0))

    def wheels_to_body(self, left_velocity: float, right_velocity: float):
        return inverse_kinematics(np.deg2rad(left_velocity), np.deg2rad(right_velocity), self.wheel)

    def body_to_wheels(self, linear: float, angular: float):
        left, right = forward_kinematics(linear, angular, self.wheel)
        return float(np.rad2deg(left)), float(np.rad2deg(right))

    def forward_cap(self, front_distance: float) -> float:
        span = self.settings.user_full_distance - self.settings.robot_full_distance
        if span <= 0.0:
            return self.settings.max_forward_velocity
        fraction = float(np.clip((front_distance - self.settings.robot_full_distance) / span, 0.0, 1.0))
        return fraction * self.settings.max_forward_velocity

    def update(
        self,
        user_wheels,
        robot_twist,
        front_distance: float,
        steer_distance: float,
        intervening: bool,
        stopped: bool,
        period: float,
        accumulate: bool,
    ) -> MixOutput:
        user_linear, user_angular = self.wheels_to_body(user_wheels[0], user_wheels[1])
        robot_linear, robot_angular = robot_twist

        weight = self.user_weight(steer_distance)
        braking = user_linear < 0.0
        mix_linear = robot_linear + (1.0 if braking else weight) * user_linear
        mix_angular = robot_angular + weight * user_angular

        if self.settings.safety_cap:
            cap = self.forward_cap(front_distance)
            if mix_linear > cap:
                mix_linear = cap
        if mix_linear < -self.settings.max_reverse_velocity:
            mix_linear = -self.settings.max_reverse_velocity
        max_angular = self.settings.max_angular_velocity
        if mix_angular > max_angular:
            mix_angular = max_angular
        elif mix_angular < -max_angular:
            mix_angular = -max_angular

        left, right = self.body_to_wheels(mix_linear, mix_angular)

        user_control = intervening and (weight >= 0.5 or braking)
        if accumulate:
            if stopped:
                self.stop_time += period
            elif user_control:
                self.user_time += period
            else:
                self.robot_time += period

        return MixOutput(
            left_velocity=left,
            right_velocity=right,
            front_distance=front_distance,
            steer_distance=steer_distance,
            user_weight=weight,
            mix_linear=mix_linear,
            mix_angular=mix_angular,
            intervening=intervening,
            user_control=user_control,
            stopped=stopped,
            user_time=self.user_time,
            robot_time=self.robot_time,
            stop_time=self.stop_time,
        )
