#!/usr/bin/env python3

import ctypes
import time
import numpy as np
import serial
from serial.tools import list_ports
from amr_teensy.protocol import (
    CHUNK_SIZE,
    STATE_STRUCT,
    STATE_VARIABLES,
    CommandType,
    ControlMode,
    DriveMotorCommand,
    GeneralCommand,
    HandleMotorCommand,
    PacketDataType,
    RxMessageType,
)
from amr_teensy.settings import DriveSettings, HandleSettings, PidGains, SerialSettings


class Teensy40:
    def __init__(self, serial_settings: SerialSettings, drive_settings: DriveSettings, handle_settings: HandleSettings):
        self.motor_setup_delay = serial_settings.motor_setup_delay

        port_name = self.find_port(serial_settings.identifier)
        if port_name is None:
            raise SystemError("Teensy not detected")

        try:
            self.serial_port = serial.Serial(
                port_name,
                baudrate=serial_settings.baudrate,
                timeout=serial_settings.timeout,
            )
            time.sleep(serial_settings.boot_delay)
        except serial.SerialException as error:
            raise SystemError(f"Cannot open serial connection to teensy: {error}")

        self.write_packet_int(CommandType.GENERAL, GeneralCommand.START_ALL)
        self.write_packet_int(CommandType.GENERAL, GeneralCommand.ENABLE_DATA_STREAM, 1)

        self.init_drive_motor(drive_settings)
        self.init_handle_motor(handle_settings.torque_limit)
        self.set_cybergear_free()

    @staticmethod
    def find_port(identifier: str):
        for port, _, hwid in list_ports.comports():
            if identifier in hwid:
                return port
        return None

    def read_packet(self):
        """
        Output
        * [RxMessageType, payload]

        Payload
        * DATA    -> ``ndarray``
        * MESSAGE -> ``str``
        * EMPTY   -> ``None``
        * INVALID -> the raw bytes that failed framing
        """
        raw = self.serial_port.read(STATE_STRUCT.size)

        if not raw:
            return RxMessageType.EMPTY, None

        if raw[:1] != b"@" or raw[-1:] != b"\n":
            return RxMessageType.INVALID, raw

        identifier = raw[1:2]
        payload = raw[2:-1]

        if identifier == b"s":
            data = np.frombuffer(payload, dtype=ctypes.c_double)
            return RxMessageType.DATA, data.reshape(CHUNK_SIZE, len(STATE_VARIABLES))

        if identifier == b"m":
            return RxMessageType.MESSAGE, payload.decode(errors="ignore").rstrip("\x00")

        return RxMessageType.INVALID, raw

    def write_packet_int(self, command_type: CommandType, command, value: int = 0):
        packet = command_type.value + command.value + PacketDataType.INT.value.pack(value) + b"\n"
        self.serial_port.write(packet)
        return packet

    def write_packet_float(self, command_type: CommandType, command, value: float = 0.0):
        packet = command_type.value + command.value + PacketDataType.DOUBLE.value.pack(value) + b"\n"
        self.serial_port.write(packet)
        return packet

    def init_drive_motor(self, drive_settings: DriveSettings):
        self.write_packet_int(CommandType.GENERAL, GeneralCommand.START_MOTORS)
        self.write_packet_int(CommandType.LEFT_MOTOR, DriveMotorCommand.SET_CONTROL_MODE, ControlMode.SPEED_CONTROL.value)
        self.write_packet_int(CommandType.RIGHT_MOTOR, DriveMotorCommand.SET_CONTROL_MODE, ControlMode.SPEED_CONTROL.value)
        self.apply_pid(CommandType.LEFT_MOTOR, drive_settings.left)
        time.sleep(self.motor_setup_delay)
        self.apply_pid(CommandType.RIGHT_MOTOR, drive_settings.right)
        time.sleep(self.motor_setup_delay)

    def apply_pid(self, motor: CommandType, gains: PidGains):
        self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_P_GAIN, gains.p_gain)
        self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_I_GAIN, gains.i_gain)
        self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_D_GAIN, gains.d_gain)

    def set_motor_speed(self, left_velocity: float, right_velocity: float):
        """
        Input
        * left_velocity (deg/s)
        * right_velocity (deg/s)
        """
        self.write_packet_float(CommandType.LEFT_MOTOR, DriveMotorCommand.SET_TARGET_SPEED, left_velocity)
        self.write_packet_float(CommandType.RIGHT_MOTOR, DriveMotorCommand.SET_TARGET_SPEED, right_velocity)

    def init_handle_motor(self, torque_limit: float):
        """
        Input
        * torque_limit (Nm)
        """
        self.write_packet_int(CommandType.MIDDLE_MOTOR, HandleMotorCommand.ENABLE_MOTOR)
        self.write_packet_int(CommandType.MIDDLE_MOTOR, HandleMotorCommand.SET_ZERO_POSITION)
        self.write_packet_float(CommandType.MIDDLE_MOTOR, HandleMotorCommand.SET_TORQUE_LIMIT, torque_limit)

    def set_cybergear_free(self):
        self.set_cybergear_torque(0.0)

    def set_cybergear_torque(self, current: float):
        """
        Input
        * current (A)
        """
        self.write_packet_int(CommandType.MIDDLE_MOTOR, HandleMotorCommand.SET_CONTROL_MODE, ControlMode.TORQUE_CONTROL.value)
        self.write_packet_float(CommandType.MIDDLE_MOTOR, HandleMotorCommand.SET_TARGET_CURRENT, current)
