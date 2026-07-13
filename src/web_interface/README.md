# web_interface

## Overview

This package serves the two webpages used to operate the robot from a phone or
laptop. The control page switches mode and authority, offers a joystick for
manual driving, handle test buttons, a sound tester, and a map where a start
pose, waypoints and a goal can be picked and sent to navigation. The log page
on its own port records the system topics into CSV files and shows their live
values. Each page has a button that jumps to the other one.

## Algorithms

- **Steady teleoperation.** Browser events update a shared state which is
  published at a fixed rate, so the robot receives a regular command stream no
  matter how irregular the browser events are.
- **Map picking.** The saved map is drawn on a canvas. Taps are converted from
  screen pixels to map coordinates. Dragging sets the heading. Waypoints are
  numbered and can be removed by tapping them again.
- **Goal dispatch.** A goal alone goes to the single-goal navigation action.
  With waypoints, the ordered list plus the goal goes to the through-poses
  action, and the robot passes through them in order. The stop button cancels
  the running action and clears the picks.
- **Arrival chime.** The result stream of both actions is watched with
  duplicate protection, so the chime plays exactly once per finished
  navigation and never spuriously at startup.
- **CSV logging.** The latest message of every logged topic is flattened into
  columns and a row is written at a fixed rate. The travelled distance is
  accumulated from the localization pose with a jump guard, and selected
  columns are written relative to their value at recording start so every file
  starts from zero.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `web_interface_node` | `web_interface_node` | Web server, teleoperation and logging |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/interface/joystick` | `custom_message/JoystickCommand` | publish | Manual driving input |
| `/interface/control` | `custom_message/Control` | publish | Mode and authority |
| `/command/sound` | `custom_message/Sound` | publish | Sound requests from the page |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | publish | Localization reset from the picked start |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | subscribe | Robot pose drawn on the map |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | action client | Single-goal navigation |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | action client | Waypoint navigation |
| logged topics | various | subscribe | The telemetry, command, detection, intersection and pose topics recorded by the logger |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `goal_reached` | `chime` | — | Sound played once when a navigation goal is reached. The names `chime` and `beep` are generated, any other name plays a wav clip, empty stays silent | — |
| `host` | `0.0.0.0` | — | Address the control page listens on, 0.0.0.0 means all network interfaces | — |
| `port` (server) | `8090` | — | Port of the control page | Change if another service uses it |
| `port` (logging) | `8091` | — | Port of the log page | Change if another service uses it |
| `rate` (logging) | `20` | Hz | CSV rows written per second while recording | Raising it gives finer time resolution and bigger files |
| `path` (logging) | `log` | — | Folder for the stored CSV files inside the package | — |
| `publish_rate` | `20` | Hz | Rate of the joystick and control publishing | Raising it reacts faster at more message traffic |
| `max_linear_velocity` | `0.3` | m/s | Speed at full joystick deflection | Raising it drives faster in manual |
| `max_angular_velocity` | `1.0` | rad/s | Turn rate at full joystick deflection | Raising it turns sharper in manual |
