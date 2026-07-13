# fastlio

> This package reuses the original FAST-LIO implementation from
> [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO), repackaged and
> configured for this workspace.

## Overview

This package estimates where the robot is and how it moves, using only the
laser scanner and its built-in motion sensor (IMU). It matches every new laser
sweep against a map it builds on the fly and combines that with the motion
sensor, producing a smooth position estimate many times per second. The rest
of the system builds on this estimate.

## Algorithms

- **Tightly coupled filtering.** An iterated error-state Kalman filter fuses
  the laser and the motion sensor in one step, instead of treating them
  separately. This keeps the estimate stable even during fast motion.
- **Motion compensation.** The points of one laser sweep are not measured at
  the same instant. The motion sensor data is used to shift every point to a
  common time, so a moving robot still produces a sharp scan.
- **Incremental map (ikd-Tree).** The local map is stored in a special search
  tree that can grow, shrink and move without being rebuilt, which makes the
  scan-to-map matching fast enough for real time.
- **Point-to-plane matching.** Each scan point is matched against a small
  plane fitted to its map neighbours, which is robust on walls and floors.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `fastlio_node` | `fastlio_node` | Laser-inertial odometry |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/livox/lidar` | `livox_driver/CustomMsg` | subscribe | Point cloud frames from the scanner |
| `/livox/imu` | `sensor_msgs/Imu` | subscribe | Motion sensor stream |
| `/fastlio/odometry` | `nav_msgs/Odometry` | publish | The estimated sensor pose |
| `/fastlio/path` | `nav_msgs/Path` | publish | The travelled path, for visualization |
| `/fastlio/cloud_registered` | `sensor_msgs/PointCloud2` | publish | Motion-corrected scan in the world frame |
| `/fastlio/cloud_registered_body` | `sensor_msgs/PointCloud2` | publish | Motion-corrected scan in the sensor frame |
| `/fastlio/cloud_effected` | `sensor_msgs/PointCloud2` | publish | Points that matched the map in the last update |
| `/fastlio/laser_map` | `sensor_msgs/PointCloud2` | publish | The accumulated map cloud |

Outputs that are disabled in the configuration are not created, and enabled
outputs are only computed while someone subscribes to them.

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `path` | `false` | — | Publish the travelled path | Turn on for visualization only |
| `cloud` | `true` | — | Publish the world frame scan | Turn off to save a little work |
| `dense_cloud` | `true` | — | Publish the full scan instead of the downsampled one | Off gives lighter messages |
| `body_cloud` | `true` | — | Publish the sensor frame scan | The scan converter needs this on |
| `effect_cloud` | `false` | — | Publish the points that matched the map | Debugging only |
| `map_cloud` | `false` | — | Publish the accumulated map | Debugging only, heavy |
| `scan_line` | `4` | lines | Number of laser lines of the device | Must match the device |
| `blind_distance` | `0.2` | m | Points closer than this are dropped | Raising it removes more returns from the robot body. Lowering it keeps closer obstacles |
| `point_skip` | `1` | — | Keep one point out of every N | Raising it saves computing power with a sparser scan. Lowering it keeps more detail |
| `feature_extract` | `false` | — | Extract plane and edge features instead of using raw points | Usually off for this type of scanner |
| `accel_covariance` | `0.3` | — | How noisy the accelerometer is assumed to be | Raising it trusts the accelerometer less. Lowering it trusts it more |
| `gyro_covariance` | `0.2` | — | How noisy the gyroscope is assumed to be | Raising it trusts the gyroscope less. Lowering it trusts it more |
| `accel_bias_covariance` | `0.0001` | — | How fast the accelerometer bias may wander | Raise if the bias drifts quickly |
| `gyro_bias_covariance` | `0.0001` | — | How fast the gyroscope bias may wander | Raise if the bias drifts quickly |
| `time_sync` | `false` | — | Software alignment of laser and motion sensor clocks | Only needed without hardware sync |
| `time_offset_lidar_to_imu` | `0.0` | s | Known constant offset between the two clocks | Set from calibration if known |
| `max_iteration` | `4` | — | Filter iterations per update | Raising it improves accuracy at more computing cost. Lowering it saves computing power |
| `scan_voxel_size` | `0.15` | m | Downsampling cell size of each new scan | Raising it saves computing power with a coarser scan. Lowering it keeps more detail, important in narrow spaces |
| `map_voxel_size` | `0.15` | m | Downsampling cell size of the map | Raising it gives a lighter map that matches faster. Lowering it keeps more detail |
| `map_cube_length` | `1000.0` | m | Side length of the region kept in memory | Raising it keeps more surroundings. Lowering it bounds memory in large areas |
| `detect_range` | `5.0` | m | The device detection range used to move the local map | Keep near the real scanner range |
| `extrinsic_estimate` | `false` | — | Refine the laser-to-IMU mounting online | Off when the mounting is precisely known |
| `extrinsic_translation` | `[-0.011, -0.02329, 0.04412]` | m | Laser position inside the device, in the IMU frame | Device constant, do not change |
| `extrinsic_rotation` | identity | — | Laser orientation in the IMU frame | Device constant, do not change |
| `runtime_log` | `false` | — | Write timing statistics on shutdown | Debugging only |
| `log_path` | `log` | — | Folder for the logs inside the package | — |
| `pcd_save` | `false` | — | Save the accumulated cloud as a file on shutdown | Heavy on disk when on |
| `pcd_save_interval` | `-1` | frames | Frames per saved file, -1 for a single file | — |
| `pcd_path` | `pcd` | — | Folder for the saved cloud files | — |
