# custom_message

## Overview

This package holds the message definitions that all the other packages use to
talk to each other. A message is a small data structure sent over a topic. The
package contains only these definitions and runs no program of its own.

| Message | What it carries |
|---|---|
| `Control` | The selected mode (manual or auto) and authority (user, robot or mix) |
| `JoystickCommand` | Manual driving input from the webpage joystick |
| `DriveCommand` | Speed commands for the left and right wheels |
| `HandleCommand` | A request to move the handle, either a nudge or a pulse |
| `Teensy` | Sensor readings from the motor board, such as wheel speeds, handle force and the emergency stop state |
| `MixControl` | Live values of the mix control, such as clearances, the user weight and the blended command |
| `Sound` | A sound request, either a named sound or the obstacle distance for the proximity beep |
| `Detection` | One detected object label from the camera |
| `DetectionArray` | A list of detections from one camera frame |
| `Intersection` | The detected junction type and which directions are open |

## Algorithms

None. This package only declares data structures.

## Nodes

| Node | Description |
|---|---|
| — | This package provides no nodes |

## Topics

| Topic | Type | Description |
|---|---|---|
| — | — | Topics are declared by the packages that use these messages |

## Parameters

| Parameter | Default | Unit | Description |
|---|---|---|---|
| — | — | — | This package has no configuration files |
