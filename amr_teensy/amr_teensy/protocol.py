#!/usr/bin/env python3

import struct
from enum import Enum

STATE_VARIABLES = (
    "stop_button",
    "motor_right_angle",  # deg
    "motor_left_angle",  # deg
    "motor_right_speed",  # deg/s
    "motor_left_speed",  # deg/s
    "motor_right_torque",  # Nm
    "motor_left_torque",  # Nm
    "cybergear_rotation_speed",  # rad/s
    "cybergear_azimuth_angle",  # rad
    "handle_force_x",
    "handle_force_y",
    "handle_force_z",
)

CHUNK_SIZE = 10

STATE_STRUCT = struct.Struct("<cc" + "d" * len(STATE_VARIABLES) * CHUNK_SIZE + "c")
COMMAND_INT_STRUCT = struct.Struct("<q")
COMMAND_DOUBLE_STRUCT = struct.Struct("<d")


class RxMessageType(Enum):
    DATA = 0
    MESSAGE = 1
    EMPTY = 2
    INVALID = 3


class CommandType(Enum):
    GENERAL = b"@"
    RIGHT_MOTOR = b">"
    LEFT_MOTOR = b"<"
    MIDDLE_MOTOR = b"*"


class GeneralCommand(Enum):
    ENABLE_DATA_STREAM = b"\x21"
    SET_RIGHT_ENCODER_ANGLE = b"\x31"
    SET_LEFT_ENCODER_ANGLE = b"\x32"
    START_ALL = b"\x41"
    START_MOTORS = b"\x42"


class ControlMode(Enum):
    TORQUE_CONTROL = 0
    SPEED_CONTROL = 1
    POSITION_CONTROL = 2
    IMPEDANCE_CONTROL = 3
    TIMED_PULSE_CONTROL = 4


class DriveMotorCommand(Enum):
    SET_CONTROL_MODE = b"\x11"
    SET_MAX_TORQUE = b"\x21"

    # Torque control
    SET_TARGET_TORQUE = b"\x31"  # torque & timed pulse control

    # Speed control
    SET_TARGET_SPEED = b"\x41"
    SET_SPEED_CONTROL_P_GAIN = b"\x42"  # speed & position control
    SET_SPEED_CONTROL_I_GAIN = b"\x43"  # speed & position control
    SET_SPEED_CONTROL_D_GAIN = b"\x44"  # speed & position control

    # Position control
    SET_TARGET_ANGLE = b"\x51"  # position & impedance control
    SET_MAX_SPEED = b"\x52"
    SET_POSITION_CONTROL_P_GAIN = b"\x53"
    SET_POSITION_CONTROL_I_GAIN = b"\x54"
    SET_POSITION_CONTROL_D_GAIN = b"\x55"

    # Impedance control
    SET_STIFFNESS = b"\x61"
    SET_DAMPING = b"\x62"
    SET_INERTIA = b"\x63"

    # Timed-pulse control
    SET_TIMED_PULSE_START_TIME = b"\x71"
    SET_TIMED_PULSE_STOP_TIME = b"\x72"
    START_TIMER = b"\x73"


class HandleMotorCommand(Enum):
    RESET_MOTOR = b"\x11"
    ENABLE_MOTOR = b"\x12"
    SET_ZERO_POSITION = b"\x13"
    SET_CONTROL_MODE = b"\x14"

    SET_TARGET_CURRENT = b"\x21"  # -23~23 A
    SET_TARGET_SPEED = b"\x22"  # -30~30 rad/s
    SET_TARGET_ANGLE = b"\x23"  # rad

    SET_TORQUE_LIMIT = b"\x31"  # 0~12 Nm, all control
    SET_CURRENT_LIMIT = b"\x32"  # 0~23 A, speed & position control
    SET_SPEED_LIMIT = b"\x33"  # 0~30 rad/s, position control

    SET_CURRENT_KP = b"\x41"  # 0.125
    SET_CURRENT_KI = b"\x42"  # 0.0158
    SET_SPEED_KP = b"\x43"  # 1.0
    SET_SPEED_KI = b"\x44"  # 0.002
    SET_POSITION_KP = b"\x45"  # 30


class PacketDataType(Enum):
    INT = COMMAND_INT_STRUCT
    DOUBLE = COMMAND_DOUBLE_STRUCT