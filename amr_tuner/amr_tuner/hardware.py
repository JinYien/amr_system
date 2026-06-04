#!/usr/bin/env python3

import ctypes
import time
import numpy as np
import serial
from serial.tools import list_ports
from amr_tuner.protocol import (
    CHUNK_SIZE,
    STATE_STRUCT,
    STATE_VARIABLES,
    CommandType,
    ControlMode,
    DriveMotorCommand,
    GeneralCommand,
    PacketDataType,
    RxMessageType,
)
from amr_tuner.settings import SerialSettings


class Teensy40:
    def __init__(self, serial_settings: SerialSettings):
        self.gain_apply_delay = serial_settings.gain_apply_delay

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

        self.init_drive_motors()

    @staticmethod
    def find_port(identifier: str):
        for port, _, hwid in list_ports.comports():
            if identifier in hwid:
                return port
        return None

    def read_packet(self):
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

    def init_drive_motors(self):
        self.write_packet_int(CommandType.GENERAL, GeneralCommand.START_MOTORS)
        self.write_packet_int(CommandType.LEFT_MOTOR, DriveMotorCommand.SET_CONTROL_MODE, ControlMode.SPEED_CONTROL.value)
        self.write_packet_int(CommandType.RIGHT_MOTOR, DriveMotorCommand.SET_CONTROL_MODE, ControlMode.SPEED_CONTROL.value)

    def apply_pid(self, p_gain: float, i_gain: float, d_gain: float):
        for motor in (CommandType.LEFT_MOTOR, CommandType.RIGHT_MOTOR):
            self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_P_GAIN, p_gain)
            self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_I_GAIN, i_gain)
            self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_D_GAIN, d_gain)
            time.sleep(self.gain_apply_delay)

    def set_motor_speed(self, left_velocity: float, right_velocity: float):
        self.write_packet_float(CommandType.LEFT_MOTOR, DriveMotorCommand.SET_TARGET_SPEED, left_velocity)
        self.write_packet_float(CommandType.RIGHT_MOTOR, DriveMotorCommand.SET_TARGET_SPEED, right_velocity)
