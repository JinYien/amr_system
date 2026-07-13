import math
import numpy as np


def matrix_from_translation_quaternion(tx, ty, tz, qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    matrix = np.identity(4)
    matrix[0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    matrix[0, 1] = 2.0 * (qx * qy - qz * qw)
    matrix[0, 2] = 2.0 * (qx * qz + qy * qw)
    matrix[1, 0] = 2.0 * (qx * qy + qz * qw)
    matrix[1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    matrix[1, 2] = 2.0 * (qy * qz - qx * qw)
    matrix[2, 0] = 2.0 * (qx * qz - qy * qw)
    matrix[2, 1] = 2.0 * (qy * qz + qx * qw)
    matrix[2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    matrix[0, 3] = tx
    matrix[1, 3] = ty
    matrix[2, 3] = tz
    return matrix


def invert_transform(matrix):
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.identity(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def planar_pose(matrix):
    x = float(matrix[0, 3])
    y = float(matrix[1, 3])
    yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    return x, y, yaw


def yaw_to_quaternion(yaw):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def planar_twist(previous, current):
    previous_x, previous_y, previous_yaw, previous_time = previous
    current_x, current_y, current_yaw, current_time = current
    elapsed = current_time - previous_time
    if elapsed <= 1e-6:
        return 0.0, 0.0
    dx = current_x - previous_x
    dy = current_y - previous_y
    forward_velocity = (dx * math.cos(current_yaw) + dy * math.sin(current_yaw)) / elapsed
    angular_velocity = normalize_angle(current_yaw - previous_yaw) / elapsed
    return forward_velocity, angular_velocity
