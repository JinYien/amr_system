# robot_system

## Overview

This is the central control node. It decides what the wheels do, based on the
mode and authority chosen on the webpage. In manual mode the webpage joystick
drives. In auto mode the user can drive with the force handle, the navigation
stack can drive alone, or both can drive together in mix, where the handle
input is added on top of the navigation command and faded out near obstacles.
The node also produces the feedback the user feels and hears, which are the
handle nudge, the handle pulse and the proximity beep.

## Algorithms

- **Force handle shaping.** The push force on the handle is smoothed, small
  noise is ignored, the rest is shaped and scaled into a forward speed with
  acceleration limits. The handle rotation angle maps to a turning speed the
  same way.
- **Wheel kinematics.** Body speeds (forward and turning) are converted to
  left and right wheel speeds and back.
- **Mix blending.** The navigation command is the baseline. The handle command
  is multiplied by a weight and added on top. The weight comes from the free
  distance in the direction the handle steers toward. Steering toward open
  space gives the user full effect, steering toward an obstacle hands control
  back to the robot. Pulling back to brake always works at full effect.
- **Safety cap.** The forward speed is capped as the free distance ahead
  shrinks, reaching zero at a configured stop distance, so the robot always
  stops for a wall ahead.
- **Stop latch.** Holding a firm pull for a short time parks the robot. It
  stays parked until the user pushes forward again.
- **Footprint distances.** All distances are measured from the edge of the
  robot body, not from its centre. When a very near obstacle drops out of the
  scanner blind zone, the last known distance is held instead of reading free.
- **Feedback.** The handle nudges back toward centre when it drifts while the
  robot drives, pulses when the steered direction is blocked in mix, and the
  front distance is sent to the sound package for the proximity beep.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `robot_system_node` | `robot_system_node` | Mode and authority control, command blending |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/interface/control` | `custom_message/Control` | subscribe | Mode and authority from the webpage |
| `/interface/joystick` | `custom_message/JoystickCommand` | subscribe | Manual driving input |
| `/teensy/telemetry` | `custom_message/Teensy` | subscribe | Handle force, handle angle and motor readings |
| `/command/robot` | `geometry_msgs/Twist` | subscribe | Speed command from navigation |
| `/livox/scan` | `sensor_msgs/LaserScan` | subscribe | Scan used to measure free distances |
| `/command/drive` | `custom_message/DriveCommand` | publish | Wheel speed commands to the motor board |
| `/command/handle` | `custom_message/HandleCommand` | publish | Nudge and pulse requests to the handle |
| `/command/user` | `geometry_msgs/Twist` | publish | The handle command alone, for logging |
| `/command/mix` | `custom_message/MixControl` | publish | Mix values for logging |
| `/command/sound` | `custom_message/Sound` | publish | Front distance for the proximity beep |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `radius` (wheel) | `0.108` | m | Drive wheel radius | Must match the real wheel or all speeds are wrong |
| `track` (wheel) | `0.42` | m | Distance between the two wheels | Must match the robot or turning is wrong |
| `frame` (footprint) | `base_link` | — | Frame the body rectangle is centred on | Must match the robot description |
| `length` (footprint) | `0.48` | m | Robot body length | Must match the real robot |
| `width` (footprint) | `0.48` | m | Robot body width | Must match the real robot |
| `period` | `0.05` | s | Control update interval | Lowering it reacts faster at more computing cost |
| `command_timeout` | `0.5` | s | Navigation commands older than this are ignored, so the robot stops when navigation goes quiet | Raising it tolerates hiccups. Lowering it stops sooner |
| `scan_timeout` | `0.5` | s | A scan older than this is treated as danger instead of trusted | Raising it tolerates scan hiccups. Lowering it fails safe sooner |
| `deadzone` (force) | `0.8` | N | Push force ignored as noise | Raising it stops creeping from a resting hand. Lowering it makes the handle more sensitive |
| `clamp_min` (force) | `-3.0` | N | Strongest pull that still counts | Widening lets stronger pulls count |
| `clamp_max` (force) | `3.0` | N | Strongest push that still counts | Widening lets stronger pushes count |
| `useful_range` (force) | `2.5` | N | Push force at which the command reaches full scale | Raising it needs a harder push for full speed. Lowering it reaches full speed with a lighter push |
| `low_pass_alpha` (force) | `0.35` | — | Smoothing of the force reading, 0 to 1 | Raising it follows the hand faster but shakier. Lowering it is smoother but slower |
| `forward_gain` (linear) | `37.5` | deg/s | Wheel speed per unit of pushed force | Raising it reaches a given speed with less push |
| `reverse_gain` (linear) | `25.5` | deg/s | Same for pulling backward | Raising it reaches reverse speed with less pull |
| `max` (linear) | `25.0` | deg/s | Forward speed ceiling of the handle | Raising it allows faster handle driving |
| `min` (linear) | `-17.0` | deg/s | Reverse speed ceiling of the handle | Widening it allows faster reverse |
| `max_acceleration` (linear) | `6.0` | deg/s² | Limit on speeding up | Raising it starts snappier. Lowering it starts gentler |
| `max_deceleration` (linear) | `15.0` | deg/s² | Limit on slowing down | Raising it stops harder when the hand releases |
| `gain` (angular) | `2.0` | — | Turning speed per unit of handle rotation | Raising it turns more for the same handle angle |
| `max_acceleration` (angular) | `90.0` | deg/s² | Limit on turn rate change | Raising it steers snappier. Lowering it steers smoother |
| `deadzone` (angular) | `0.02` | rad | Handle angle ignored as noise | Raising it stops wandering when driving straight. Lowering it steers finer |
| `obstacle_cone` | `50.0` | deg | Half angle of the front window used for the speed cap and the beep | Raising it also slows for obstacles further to the sides |
| `steer_cone` | `20.0` | deg | Half angle of the window around the steered direction used for the user weight | Raising it considers obstacles near the steered path. Lowering it checks only exactly where the handle points |
| `user_full_distance` | `0.7` | m | Free distance at which the user has full authority | Raising it hands control to the robot from further out |
| `robot_full_distance` | `0.2` | m | Free distance at which the robot has full authority and the speed cap reaches zero | Raising it stops further from obstacles. Lowering it allows a closer approach |
| `blind_latch_distance` | `0.4` | m | When a nearer obstacle leaves the scanner blind zone, the last distance is held | Raising it catches faster approaches. Lowering it releases the hold sooner |
| `max_forward_velocity` | `0.8` | m/s | Forward speed allowed when the path is clear in mix | Raising it allows a faster pushed boost |
| `max_reverse_velocity` | `0.1` | m/s | Reverse speed limit in mix | Kept low because the scanner does not see behind the robot |
| `max_angular_velocity` | `1.0` | rad/s | Turn rate limit of the blended command | Raising it allows sharper blended turns |
| `stop_hold_time` | `0.5` | s | How long a firm pull must be held to park the robot | Raising it avoids accidental stops. Lowering it parks with a shorter pull |
| `safety_cap` | `true` | — | Cap the forward speed to zero at the stop distance | Off relies only on the weight fade, not recommended |
| `align_angle` (nudge) | `10.0` | deg | Handle drift allowed before it is nudged back to centre | Raising it allows more drift before nudging |
| `repeat_interval` (nudge) | `2.0` | s | Time between repeated nudges | Raising it gives calmer cues |
| `distance` (pulse) | `1.0` | m | Free distance below which the handle pulses as a blocked warning | Raising it warns earlier |
| `repeat_interval` (pulse) | `0.5` | s | Time between pulses | Keep above the pulse pattern length of about 0.3 s |

The launch files in this package also pin every node of the system to a CPU
core. The core plan is documented in the root README.
