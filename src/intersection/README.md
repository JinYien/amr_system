# intersection

## Overview

This node looks at the laser scan and recognizes what kind of place the robot
is in. It watches three windows, ahead, left and right, decides which of them
are open passages, and names the junction, for example corridor, left corner,
T-junction, cross, dead end or open space. When the junction type changes it
can play a sound. The node is optional and is commented out of the launch by
default.

## Algorithms

- **Sector split.** Every scan beam is assigned to the front, left or right
  window by its angle.
- **Gap finding.** Free beams between two wall returns form a candidate
  opening. The width of the opening is computed from the geometry of its two
  bounding walls. A window counts as open when an opening at least as wide as
  the robot plus a safety margin exists beyond the open distance.
- **Footprint distances.** The configured distances mean clearance from the
  edge of the robot body. The node adds the half body size internally because
  the scanner measures from the robot centre.
- **Debouncing.** A window must stay open or closed for a hold time before its
  state switches, so the result does not flicker.
- **Junction naming.** The three open flags map to a junction type through a
  fixed table. When everything is open and the nearest wall is far, the label
  switches to open space, with a hysteresis band so it does not flicker at the
  boundary.
- **Change sounds.** A clip plays once per junction change. It stays silent in
  auto mode with user authority.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `intersection_node` | `intersection_node` | Junction recognition and announcements |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/livox/scan` | `sensor_msgs/LaserScan` | subscribe | Flat scan input |
| `/interface/control` | `custom_message/Control` | subscribe | Mode and authority, used to mute sounds |
| `/intersection/state` | `custom_message/Intersection` | publish | Junction type and open flags |
| `/intersection/markers` | `visualization_msgs/MarkerArray` | publish | Windows and openings drawn for RViz |
| `/command/sound` | `custom_message/Sound` | publish | Junction change announcements |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `enable` (sounds) | `true` | — | Play junction sounds | Off stays silent |
| `deadend` … `open_space` | clip names | — | Wav clip played for each junction type | Empty entries stay silent |
| `width` (robot) | `0.48` | m | Robot body width | An opening must beat width plus twice the margin to count |
| `safety_margin` | `0.05` | m | Extra width required on each side of an opening | Raising it only accepts wider passages |
| `lidar_yaw_offset` | `0.0` | deg | Rotation from the scan frame to the robot frame | Keep 0, the scan is already robot-centred |
| `min_open_distance` | `1.5` | m | Clearance from the body edge beyond which a beam counts as free | Raising it only flags openings that lead somewhere. Lowering it reacts sooner |
| `open_space_distance` | `2.5` | m | Nearest wall distance beyond which everything-open becomes open space | Raising it needs a larger area to call open space |
| `open_space_hysteresis` | `0.2` | m | Stickiness of the open space label | Raising it flickers less at the boundary |
| `front_min` / `front_max` | `-40` / `40` | deg | Front window arc | Widening watches a broader front |
| `left_min` / `left_max` | `50` / `130` | deg | Left window arc | — |
| `right_min` / `right_max` | `-130` / `-50` | deg | Right window arc | — |
| `debounce_time` | `0.2` | s | Hold time before an open flag may switch | Raising it is steadier but reacts slower |
| `detection_rate` | `5.0` | Hz | How often the classification runs | Raising it reacts faster at more computing |
| `enable` (visualization) | `true` | — | Publish the RViz markers | Only computed while subscribed |
| `marker_lifetime` | `0.3` | s | How long a marker stays visible | Raise if markers blink in RViz |
| `sector_alpha` | `0.25` | — | Transparency of the window overlays | Raising it is more opaque |
| `point_size` | `0.04` | m | Size of the drawn scan points | — |
