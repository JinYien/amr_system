import numpy as np
from robot_system.settings import AngularSettings, ForceSettings, LinearSettings, WheelSettings
from robot_system.differential_drive import forward_kinematics


class ForceCommand:
    def __init__(
        self, control_period: float, force: ForceSettings, linear: LinearSettings, angular: AngularSettings, wheel: WheelSettings
    ):
        self.control_period = control_period
        self.force = force
        self.linear = linear
        self.angular = angular
        self.wheel = wheel

        self.force_filtered = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    @staticmethod
    def soft_deadzone(value: float, threshold: float) -> float:
        if value > threshold:
            return value - threshold
        if value < -threshold:
            return value + threshold
        return 0.0

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def rate_limit(self, target: float, current: float, max_acceleration: float) -> float:
        max_step = max_acceleration * self.control_period
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def rate_limit_linear(self, target: float, current: float) -> float:
        delta = target - current
        decelerating = (current > 0.0 and delta < 0.0) or (current < 0.0 and delta > 0.0)
        max_acceleration = self.linear.max_deceleration if decelerating else self.linear.max_acceleration
        max_step = max_acceleration * self.control_period
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def update(self, raw_force: float, raw_azimuth_angle: float):
        self.force_filtered += self.force.low_pass_alpha * (raw_force - self.force_filtered)

        force = self.clamp(self.force_filtered, self.force.clamp_min, self.force.clamp_max)
        force = self.soft_deadzone(force, self.force.deadzone)

        useful = self.force.useful_range - self.force.deadzone
        normalized = 0.0 if useful <= 0.0 else force / useful

        shape = float(np.tanh(normalized))
        gain = self.linear.forward_gain if shape >= 0.0 else self.linear.reverse_gain
        linear_target = self.clamp(gain * shape, self.linear.min, self.linear.max)

        self.linear_velocity = self.rate_limit_linear(linear_target, self.linear_velocity)
        self.linear_velocity = self.clamp(self.linear_velocity, self.linear.min, self.linear.max)

        if abs(raw_azimuth_angle) <= self.angular.deadzone:
            self.angular_velocity = 0.0
        else:
            angular_target = -self.angular.gain * np.rad2deg(raw_azimuth_angle)
            self.angular_velocity = self.rate_limit(angular_target, self.angular_velocity, self.angular.max_acceleration)

        left_velocity, right_velocity = forward_kinematics(self.linear_velocity, self.angular_velocity, self.wheel)
        return float(left_velocity), float(right_velocity)
