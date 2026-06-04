#!/usr/bin/env python3

import struct
from enum import Enum

STATE_VARIABLES = (
    "stop_button",
    "motor_right_angle",
    "motor_left_angle",
    "motor_right_speed",
    "motor_left_speed",
    "motor_right_torque",
    "motor_left_torque",
    "cybergear_rotation_speed",
    "cybergear_azimuth_angle",
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
    START_ALL = b"\x41"
    START_MOTORS = b"\x42"


class ControlMode(Enum):
    TORQUE_CONTROL = 0
    SPEED_CONTROL = 1
    POSITION_CONTROL = 2


class DriveMotorCommand(Enum):
    SET_CONTROL_MODE = b"\x11"
    SET_TARGET_SPEED = b"\x41"
    SET_SPEED_CONTROL_P_GAIN = b"\x42"
    SET_SPEED_CONTROL_I_GAIN = b"\x43"
    SET_SPEED_CONTROL_D_GAIN = b"\x44"


class PacketDataType(Enum):
    INT = COMMAND_INT_STRUCT
    DOUBLE = COMMAND_DOUBLE_STRUCT
