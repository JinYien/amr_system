#!/usr/bin/env python3

import math
import numpy as np
from amr_teensy.settings import WheelSettings


def inverse_kinematics(left_velocity: float, right_velocity: float, wheel: WheelSettings):
    """
    Input
    * left_velocity (rad/s)
    * right_velocity (rad/s)

    Output
    * linear_velocity (m/s)
    * angular_velocity (rad/s)
    """
    linear_velocity = wheel.radius * (left_velocity + right_velocity) / 2.0
    angular_velocity = wheel.radius * (right_velocity - left_velocity) / wheel.track
    return linear_velocity, angular_velocity


def pose_derivative(left_velocity: float, right_velocity: float, heading_angle: float, wheel: WheelSettings) -> np.ndarray:
    """
    Input
    * left_velocity (rad/s)
    * right_velocity (rad/s)
    * heading_angle (rad)

    Output
    * [dx/dt, dy/dt, dθ/dt] (m/s, m/s, rad/s)
    """
    linear = wheel.radius * (left_velocity + right_velocity) / 2.0
    return np.array(
        [
            linear * math.cos(heading_angle),
            linear * math.sin(heading_angle),
            wheel.radius * (right_velocity - left_velocity) / wheel.track,
        ]
    )


def update_pose(
    pose: np.ndarray, timestep: float, left_velocity: float, right_velocity: float, wheel: WheelSettings
) -> np.ndarray:
    """
    Input
    * pose [x, y, θ] (m, m, rad)
    * timestep (s)
    * left_velocity (rad/s)
    * right_velocity (rad/s)

    Output
    * [x, y, θ] (m, m, rad)
    """
    heading_angle = pose[2]
    k1 = pose_derivative(left_velocity, right_velocity, heading_angle, wheel)
    k2 = pose_derivative(left_velocity, right_velocity, heading_angle + k1[2] * timestep / 2.0, wheel)
    k3 = pose_derivative(left_velocity, right_velocity, heading_angle + k2[2] * timestep / 2.0, wheel)
    k4 = pose_derivative(left_velocity, right_velocity, heading_angle + k3[2] * timestep, wheel)
    return pose + (timestep / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def torque_to_wrench(left_torque: float, right_torque: float, wheel: WheelSettings):
    """
    Input
    * left_torque (Nm)
    * right_torque (Nm)

    Output
    * linear_force (N)
    * rotational_moment (Nm)
    """
    linear_force = (left_torque + right_torque) / wheel.radius
    rotational_moment = (right_torque - left_torque) * wheel.track / (2.0 * wheel.radius)
    return linear_force, rotational_moment


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """
    Input
    * roll (rad)
    * pitch (rad)
    * yaw (rad)

    Output
    * [x, y, z, w]
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw
