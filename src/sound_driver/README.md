# sound_driver

## Overview

This node is the speaker of the robot. It plays three kinds of sound. Named
wav clips announce things like detected objects. Two generated sounds, a short
test beep and a rising chime, need no files at all. And a continuous proximity
beep speeds up as the robot gets closer to an obstacle, exactly like a car
parking sensor, ending in one solid tone when very close.

## Algorithms

- **Continuous beep stream.** One long sine tone is generated piece by piece
  and fed to a single audio process a little ahead of real time. The beeping
  rhythm is made by switching this one tone on and off with soft fades, so the
  rhythm can change smoothly with distance and there are never clicks or gaps
  between beeps.
- **Distance to rhythm.** Between the far and near distances, the beep period
  shrinks smoothly from the far interval to the near interval. Beyond the far
  distance the beep is silent. At or below the near distance the tone stays on
  solid.
- **Generated sounds.** The test beep and the three-note chime are synthesized
  as short tone samples with fade envelopes.
- **Playback arbitration.** While a clip is playing the proximity beep pauses,
  and a new clip is not started until the previous one finished.

## Nodes

| Node | Executable | Description |
|---|---|---|
| `sound_node` | `sound_driver_node` | Clip, chime and proximity beep playback |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/command/sound` | `custom_message/Sound` | subscribe | Sound requests. A clip name plays that wav file, the names `beep` and `chime` play the generated sounds, and a message with only a distance drives the proximity beep |

## Parameters

### `config/config.yaml`

| Parameter | Default | Unit | What it does | Raising / lowering it |
|---|---|---|---|---|
| `near_distance` | `0.2` | m | At or below this distance the beep becomes one solid tone | Raising it reaches the solid tone earlier |
| `far_distance` | `0.4` | m | Beyond this distance the beep is silent | Raising it starts beeping further from obstacles |
| `near_interval` | `0.1` | s | Beep period at the near end | Lowering it makes the close-up beeping more frantic |
| `far_interval` | `2.0` | s | Beep period at the far end | Raising it makes distant obstacles tick slower |
| `gap` | `0.05` | s | Silent part inside each beep period | Raising it makes the beeps more separated |
| `frequency` | `880.0` | Hz | Pitch of the beep tone | Raising it sounds higher |
| `volume` | `0.6` | — | Output level from 0.0 to 1.0 | Raising it is louder |

Wav clips live in the package `wav` folder and are played by their file name
without the extension.
