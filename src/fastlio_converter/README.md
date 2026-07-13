# fastlio_converter

## Overview

This node turns the 3D point cloud from the laser package into a simple 2D
laser scan, as if the robot had a flat laser scanner mounted at its centre.
Localization, mapping, the navigation costmaps and the control node all read
this one scan, so they all see the same obstacles.

## Algorithms

- **Frame transform.** Every point is moved from the scanner frame into the
  robot base frame using the fixed mounting transform, so the scan is always
  centred on the robot and 0 degrees is always straight ahead.
- **Height slice.** Only points inside a height band above the floor are kept.
  This keeps obstacles at body height and removes the floor and the ceiling.
- **Body removal.** Points that fall inside the robot body rectangle (plus a
  small margin) are removed, so the robot never mistakes itself for an
  obstacle.
- **Polar binning.** The remaining points are sorted into angle bins. Each bin
  keeps only its nearest point. Bins with no return are published as infinity,
  which consumers treat as free space.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `fastlio_converter_node` | `fastlio_converter_node` | 3D point cloud to flat 2D laser scan |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/fastlio/cloud_registered_body` | `sensor_msgs/PointCloud2` | subscribe | Motion-corrected cloud in the sensor frame |
| `/livox/scan` | `sensor_msgs/LaserScan` | publish | Flat scan in the robot base frame |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `base` (frames) | `base_link` | — | Frame the scan is expressed in | Must match the robot description |
| `sensor` (frames) | `livox` | — | Scanner frame used for the mounting lookup | Must match the robot description |
| `min_height` | `0.0` | m | Lowest point kept, above the robot base | Raising it ignores low obstacles. Lowering it sees lower but may catch the floor |
| `max_height` | `0.5` | m | Highest point kept | Raising it sees taller obstacles with more noise. Lowering it keeps a cleaner slice |
| `range_min` | `0.27` | m | Points closer than this are dropped | Raising it ignores more near returns. Lowering it sees closer, but must stay below 0.29 or the walls of the narrowest gap disappear while inside it |
| `range_max` | `30.0` | m | Points farther than this are dropped | Raising it maps farther. Lowering it drops far noise |
| `angle_min` | `-130.0` | deg | Start of the kept bearing arc | Widening toward -180 sees more around the robot |
| `angle_max` | `130.0` | deg | End of the kept bearing arc | Widening toward 180 sees more around the robot |
| `angle_increment` | `0.5` | deg | Width of one angle bin | Raising it gives fewer bins and less computing. Lowering it gives a finer scan |
| `length` (footprint) | `0.48` | m | Robot body length, interior points are removed | Must match the real robot |
| `width` (footprint) | `0.48` | m | Robot body width | Must match the real robot |
| `margin` (footprint) | `0.02` | m | Extra clearance carved around the body | Raising it removes lingering body edges. Lowering it keeps closer obstacles. Body half plus margin must stay below 0.29 or narrow gap walls get deleted |
