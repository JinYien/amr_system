# teensy_driver

## Overview

This node talks to the microcontroller board that physically drives the
motors. It sends wheel speed commands and handle motor patterns down the
serial cable, and publishes what the board reports back, such as wheel speeds,
the handle force and angle, and the emergency stop button. If commands stop
arriving it stops the wheels on its own.

## Algorithms

- **Serial protocol.** Framed packets are read in a fast loop, decoded into
  named telemetry fields and republished as one message.
- **Command watchdog.** If no wheel command arrives within a timeout the wheel
  speeds are forced to zero, so a crashed upstream node can never leave the
  robot driving.
- **Emergency stop.** When the hardware stop button is pressed, the wheels are
  zeroed and the handle motor is released.
- **Handle patterns.** A nudge request becomes a short one-sided torque push,
  a pulse request becomes a quick alternating torque sequence, both bounded by
  a torque limit.
- **Motor setup.** The wheel controller gains are written to the board once at
  startup.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `teensy_node` | `teensy_driver_node` | Serial bridge to the motor board |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/teensy/telemetry` | `custom_message/Teensy` | publish | Sensor readings from the board |
| `/command/drive` | `custom_message/DriveCommand` | subscribe | Wheel speed commands |
| `/command/handle` | `custom_message/HandleCommand` | subscribe | Nudge and pulse requests |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `usb_id` | `VID:PID=16C0:0483` | — | USB identifier used to find the board port | Must match the board |
| `baudrate` | `115200` | baud | Serial connection speed | Must match the board firmware |
| `read_timeout` | `0.01` | s | Wait limit for one serial read | Raising it tolerates a slow stream but blocks longer. Lowering it returns faster with possibly incomplete packets |
| `boot_delay` | `1.0` | s | Wait after opening the port before talking to the board | Raise if the board misses the first commands |
| `motor_setup_delay` | `0.5` | s | Wait after sending each motor gain setup | Raise if gains are not applied reliably |
| `serial_read` | `1000` | Hz | Telemetry polling rate | Raising it loses fewer packets at more computing cost |
| `drive_command` | `100` | Hz | Rate at which wheel commands are written to the board | Raising it reacts faster on the wheels |
| `handle_command` | `50` | Hz | Rate of the handle motor updates | Raising it makes patterns crisper |
| `command_timeout` | `0.5` | s | Zero the wheels when no command arrives within this time | Raising it tolerates hiccups. Lowering it stops sooner when upstream dies |
| `p_gain` (left, right) | `0.0810` | — | Proportional gain of the wheel speed controller | Raising it follows commands harder but can oscillate |
| `i_gain` (left, right) | `0.8005` | — | Integral gain of the wheel speed controller | Raising it removes steady error faster but can overshoot |
| `d_gain` (left, right) | `0.0` | — | Derivative gain of the wheel speed controller | Raising it damps oscillation but amplifies noise |
| `torque_limit` (middle) | `1.0` | Nm | Ceiling of the handle motor torque | Raising it allows stronger handle cues |
| `torque` (pulse) | `1.5` | Nm | Strength of one pulse step | Raising it makes the pulse more noticeable |
| `hold_time` (pulse) | `0.05` | s | Duration of one pulse step | Raising it makes the pattern slower and longer |
| `sequence` (pulse) | `[0, 1, -1, 1, -1, 0]` | — | Direction pattern of the pulse | More entries make a longer buzz |
| `torque` (nudge) | `1.5` | Nm | Strength of the nudge push | Raising it makes the nudge firmer |
| `hold_time` (nudge) | `0.10` | s | Duration of the nudge push | Raising it pushes the handle further |
