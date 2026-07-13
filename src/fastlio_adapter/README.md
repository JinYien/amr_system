# fastlio_adapter

## Overview

The odometry from the laser package describes where the **scanner** is, in
full 3D. The rest of the system wants to know where the **robot base** is, on
a flat floor. This small node does that translation. It shifts the pose from
the scanner to the centre of the robot, flattens it to x, y and heading,
starts it at zero, and publishes it as the standard odometry that mapping and
navigation expect.

## Algorithms

- **Mounting compensation.** The fixed position and angle of the scanner on
  the robot is read once from the robot description. Its inverse is applied to
  every incoming pose, so the output describes the robot base no matter where
  the scanner is mounted.
- **First pose zeroing.** The very first pose is remembered and subtracted
  from all later ones, so the odometry always starts at zero where the robot
  powered on.
- **Flattening.** Only x, y and the heading angle are kept. Height, roll and
  pitch are dropped because the robot drives on a flat floor.
- **Velocity from differences.** Forward and turning speed are computed from
  the change between consecutive poses.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `fastlio_adapter_node` | `fastlio_adapter_node` | Scanner odometry to robot base odometry |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/fastlio/odometry` | `nav_msgs/Odometry` | subscribe | Scanner pose from the odometry package |
| `/odom` | `nav_msgs/Odometry` | publish | Flat odometry of the robot base |
| `/tf` | `tf2_msgs/TFMessage` | publish | The odom to base_link transform |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `odom` (frames) | `odom` | — | Name of the odometry frame on the output | Must match the navigation configuration |
| `base` (frames) | `base_link` | — | Name of the robot base frame | Must match the robot description |
| `sensor` (frames) | `livox` | — | Name of the scanner frame, used to look up the mounting offset | Must match the robot description |
| `zero_first_pose` | `true` | — | Start the odometry at zero from the first received pose | Off keeps the origin of the odometry package instead |
| `publish_tf` | `true` | — | Broadcast the odom to base transform | Mapping and navigation need this on unless another node provides it |
