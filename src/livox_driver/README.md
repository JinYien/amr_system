# livox_driver

> This package reuses the original Livox ROS 2 driver from
> [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2),
> repackaged with a simplified configuration and built-in point filters.

## Overview

This is the driver for the Livox MID360 laser scanner. The scanner constantly
measures distances all around the robot and also contains a motion sensor
(IMU). The driver receives this data over the network cable, groups the
measured points into frames, throws away points that are too close, too far,
too low or too high, and publishes the result for the rest of the system.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `livox_driver_node` | `livox_driver_node` | Scanner driver with point filters |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/livox/lidar` | `livox_driver/CustomMsg` or `sensor_msgs/PointCloud2` | publish | The measured points, grouped into frames |
| `/livox/imu` | `sensor_msgs/Imu` | publish | The motion sensor stream of the device |

## Algorithms

- **Frame packing.** Raw packets from the device are collected and packed into
  one message per publish period. Every point keeps its own timestamp so later
  processing can correct for motion during the sweep.
- **Point filtering.** Every point is checked against a distance band and a
  height band before it is published. Points from the robot itself and noisy
  far returns are removed at the source, which lightens every consumer.
- **Two output formats.** The cloud can be published as a standard point cloud
  or as the Livox custom format with per-point timestamps, which the odometry
  package needs.

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `device_type` | `MID360` | — | The Livox model connected to the driver | Must match the real device |
| `point_cloud_format` | `1` | — | Output format. 0 is a standard point cloud, 1 is the Livox custom format, 2 is a PCL cloud | The odometry package needs format 1 |
| `publish_frequency` | `60.0` | Hz | How often the collected points are packed into a message | Raising it gives fresher and smaller messages at more overhead. Lowering it gives fewer but bigger messages |
| `separate_topic_per_lidar` | `false` | — | Publish one topic per device instead of a shared one | Only matters with more than one scanner |
| `frame_id` | `livox` | — | The coordinate frame name stamped on the messages | Must match the scanner frame in the robot description |
| `min_range` | `0.1` | m | Points closer than this are dropped | Raising it removes more returns from the robot body. Lowering it keeps closer points |
| `max_range` | `5.0` | m | Points farther than this are dropped | Raising it keeps farther returns for mapping. Lowering it drops far noise and lightens the whole system |
| `min_height` | `-0.30` | m | Lowest point kept, measured in the scanner frame | Raising it removes more floor returns. Lowering it keeps lower obstacles |
| `max_height` | `1.20` | m | Highest point kept, measured in the scanner frame | Raising it sees taller obstacles. Lowering it cuts ceiling returns |
