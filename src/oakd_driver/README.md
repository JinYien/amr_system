# oakd_driver

## Overview

This is the driver for the OAK-D camera. The camera runs an object detection
neural network on its own built-in chip, so the main computer stays free. The
node publishes which objects are seen, can publish a camera image with the
detections drawn on it, and announces newly seen objects with a sound while
the system drives in auto mode.

## Algorithms

- **On-device inference.** The detection network runs inside the camera. The
  computer only receives the small detection results and preview frames.
- **Confidence filter.** Detections the network is not sure about are dropped
  inside the camera before they ever reach the computer.
- **Announcement debounce.** Every object label is announced once. It is not
  announced again until it has been out of view for a forget time, so the
  robot does not repeat itself while an object stays visible.
- **Auto mode gate.** Announcements only play in auto mode. Manual driving
  stays quiet.
- **Automatic reconnect.** If the camera connection drops, the node keeps
  retrying in the background and rebuilds the pipeline when it returns.
- **Lazy imaging.** The annotated image and its compressed copy are only
  produced while someone is actually subscribed to them.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `camera_node` | `oakd_driver_node` | Camera driver with on-device detection |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/oakd/detection` | `custom_message/DetectionArray` | publish | Labels of the detected objects |
| `/oakd/image` | `sensor_msgs/Image` | publish | Camera image with detections drawn on it |
| `/oakd/image/compressed` | `sensor_msgs/CompressedImage` | publish | JPEG copy for viewing over the network |
| `/command/sound` | `custom_message/Sound` | publish | One announcement per newly seen object |
| `/interface/control` | `custom_message/Control` | subscribe | Mode used to gate the announcements |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `enable` (sounds) | `true` | — | Play detection announcements | Off stays silent |
| `forget_time` | `3.0` | s | How long an object must be gone before it is announced again | Raising it repeats the same object less often. Lowering it re-announces sooner |
| `fps` | `30` | Hz | Camera and inference frame rate | Raising it detects faster at more USB and compute load. Lowering it lightens the camera |
| `resolution` | `auto` | — | Sensor resolution, auto picks the device default | Higher resolutions cost more USB bandwidth |
| `keep_aspect_ratio` | `false` | — | Crop instead of stretch when resizing for the network | On keeps shapes undistorted but loses the frame edges |
| `reconnect_delay` | `1.0` | s | Wait between camera reconnect attempts | Raising it retries less often. Lowering it recovers faster |
| `blob_path` | `model/best.superblob` | — | Compiled network file inside the package | — |
| `config_path` | `model/config.json` | — | Network description file | Must match the blob |
| `shaves` | `6` | cores | Camera compute cores used for the network | Raising it infers faster if cores are free |
| `confidence_threshold` | `0.8` | — | Minimum confidence a detection must have, 0 to 1 | Raising it gives fewer false alarms but more misses. Lowering it catches more objects with more noise |
| `enable` (visualization) | `false` | — | Create the image publishers | Images are only produced while subscribed either way |
| `publish_rate` | `30` | Hz | Upper limit on the published image rate | Raising it streams smoother at more network load |
| `jpeg_quality` | `80` | — | Quality of the compressed image, 0 to 100 | Raising it is sharper at more bandwidth |
