#!/usr/bin/env python3

from amr_system.settings import WheelSettings


def forward_kinematics(linear_velocity: float, angular_velocity: float, wheel: WheelSettings):
    """
    Input:
    * linear_velocity (m/s)
    * angular_velocity (rad/s)

    Output:
    * left_wheel_velocity (rad/s)
    * right_wheel_velocity (rad/s)
    """
    left_wheel_velocity = (linear_velocity - angular_velocity * wheel.track / 2.0) / wheel.radius
    right_wheel_velocity = (linear_velocity + angular_velocity * wheel.track / 2.0) / wheel.radius
    return left_wheel_velocity, right_wheel_velocity
