import ctypes
import time
import numpy as np
import serial
from serial.tools import list_ports
from teensy_driver.protocol import (
    SAMPLES_PER_PACKET,
    PACKET_STRUCT,
    TELEMETRY_FIELDS,
    CommandType,
    ControlMode,
    DriveMotorCommand,
    GeneralCommand,
    HandleMotorCommand,
    PacketDataType,
    RxMessageType,
)
from teensy_driver.settings import MotorSettings, PidGains, SerialSettings


class Teensy40:
    def __init__(self, serial_settings: SerialSettings, motor_settings: MotorSettings):
        self.motor_setup_delay = serial_settings.motor_setup_delay
        self.handle_torque_mode_set = False

        port = self.find_serial_port(serial_settings.usb_id)
        if port is None:
            raise SystemError(f"Teensy board not found, no serial device matches usb_id {serial_settings.usb_id}")

        try:
            self.serial_port = serial.Serial(port, baudrate=serial_settings.baudrate, timeout=serial_settings.read_timeout)
            time.sleep(serial_settings.boot_delay)
        except serial.SerialException as error:
            raise SystemError(f"Teensy board found on {port} but the serial port could not be opened: {error}")

        self.write_packet_int(CommandType.GENERAL, GeneralCommand.START_ALL)
        self.write_packet_int(CommandType.GENERAL, GeneralCommand.ENABLE_DATA_STREAM, 1)

        self.init_drive_motor(motor_settings)
        self.init_handle_motor(motor_settings.middle.torque_limit)
        self.free_handle_motor()

    @staticmethod
    def find_serial_port(usb_id: str):
        for port, description, hardware_id in list_ports.comports():
            if usb_id in hardware_id:
                return port
        return None

    def read_packet(self):
        raw = self.serial_port.read(PACKET_STRUCT.size)

        if not raw:
            return RxMessageType.EMPTY, None

        if len(raw) < PACKET_STRUCT.size or raw[:1] != b"@" or raw[-1:] != b"\n":
            self.serial_port.read_until(b"\n")
            return RxMessageType.INVALID, raw

        identifier = raw[1:2]
        payload = raw[2:-1]

        if identifier == b"s":
            data = np.frombuffer(payload, dtype=ctypes.c_double)
            return RxMessageType.DATA, data.reshape(SAMPLES_PER_PACKET, len(TELEMETRY_FIELDS))

        if identifier == b"m":
            return RxMessageType.MESSAGE, payload.decode(errors="ignore").rstrip("\x00")

        return RxMessageType.INVALID, raw

    def write_packet_int(self, command_type: CommandType, command, value: int = 0):
        packet = command_type.value + command.value + PacketDataType.INT.value.pack(value) + b"\n"
        self.serial_port.write(packet)

    def write_packet_float(self, command_type: CommandType, command, value: float = 0.0):
        packet = command_type.value + command.value + PacketDataType.DOUBLE.value.pack(value) + b"\n"
        self.serial_port.write(packet)

    def init_drive_motor(self, motor_settings: MotorSettings):
        self.write_packet_int(CommandType.GENERAL, GeneralCommand.START_MOTORS)
        self.write_packet_int(CommandType.LEFT_MOTOR, DriveMotorCommand.SET_CONTROL_MODE, ControlMode.SPEED_CONTROL.value)
        self.write_packet_int(CommandType.RIGHT_MOTOR, DriveMotorCommand.SET_CONTROL_MODE, ControlMode.SPEED_CONTROL.value)
        self.apply_pid(CommandType.LEFT_MOTOR, motor_settings.left)
        time.sleep(self.motor_setup_delay)
        self.apply_pid(CommandType.RIGHT_MOTOR, motor_settings.right)
        time.sleep(self.motor_setup_delay)

    def apply_pid(self, motor: CommandType, gains: PidGains):
        self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_P_GAIN, gains.p_gain)
        self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_I_GAIN, gains.i_gain)
        self.write_packet_float(motor, DriveMotorCommand.SET_SPEED_CONTROL_D_GAIN, gains.d_gain)

    def set_drive_speed(self, left_velocity: float, right_velocity: float):
        self.write_packet_float(CommandType.LEFT_MOTOR, DriveMotorCommand.SET_TARGET_SPEED, left_velocity)
        self.write_packet_float(CommandType.RIGHT_MOTOR, DriveMotorCommand.SET_TARGET_SPEED, right_velocity)

    def init_handle_motor(self, torque_limit: float):
        self.write_packet_int(CommandType.MIDDLE_MOTOR, HandleMotorCommand.ENABLE_MOTOR)
        self.write_packet_int(CommandType.MIDDLE_MOTOR, HandleMotorCommand.SET_ZERO_POSITION)
        self.write_packet_float(CommandType.MIDDLE_MOTOR, HandleMotorCommand.SET_TORQUE_LIMIT, torque_limit)

    def free_handle_motor(self):
        self.set_handle_torque(0.0)

    def set_handle_torque(self, torque: float):
        if not self.handle_torque_mode_set:
            self.write_packet_int(
                CommandType.MIDDLE_MOTOR,
                HandleMotorCommand.SET_CONTROL_MODE,
                ControlMode.TORQUE_CONTROL.value,
            )
            self.handle_torque_mode_set = True
        self.write_packet_float(CommandType.MIDDLE_MOTOR, HandleMotorCommand.SET_TARGET_CURRENT, torque)
