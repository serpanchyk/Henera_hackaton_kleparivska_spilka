# Two-LED CV Decoder Plan

## Confirmed Architecture

- LED commands are sent to `/model/x500_mono_cam_{id}/led_cmd`.
- Direct Gazebo commands use `gz.msgs.StringMsg`.
- ROS-side commands use `std_msgs/msg/String` bridged to `gz.msgs.StringMsg`.
- Camera frames are published at `/world/baylands_custom/model/x500_mono_cam_{id}/link/mono_cam/base_link/sensor/camera_sensor/image`.
- The camera model is `640x480`, `30 FPS`, with horizontal FOV `1.6` radians.

The Gazebo LED plugin finds visuals named `led_lens_*`, sorts them by name, and applies a four-character binary mask. The current model defines `led_lens_01` and `led_lens_04`. The active two-lens protocol uses bit 1 for the fixed green LED and bit 2 for the fixed red LED; bits 3 and 4 are unused compatibility padding:

| Mask | Meaning |
| --- | --- |
| `1100` | green ON, red ON |
| `1000` | green ON, red OFF |
| `0100` | green OFF, red ON |
| `0000` | green OFF, red OFF |

LED color is fixed by lens role: `led_lens_01` is green when enabled, `led_lens_04` is red when enabled, and disabled LEDs are dark.

## LED Protocol

- `led_lens_01` is the green LED.
- `led_lens_04` is the red LED.
- `FOLLOW`: green + red ON, mask `1100`.
- `HOLD`: green ON only, mask `1000`.
- `SAFE`: red ON only, mask `0100`.
- `FINISH`: green + red blink together in phase, alternating `1100` and `0000` at about 1 Hz.

The shared protocol constants and timing helpers live in `drone_sdk/two_led_cv/protocol.py`.

## Observation Contract

The detector should return `TwoLedObservation` from `drone_sdk/two_led_cv/types.py`:

```python
@dataclass
class TwoLedObservation:
    visible: bool
    x_error: float | None
    y_error: float | None
    led_distance_px: float | None
    pair_angle_rad: float | None
    anchor_visible: bool
    signal_visible: bool
    state: str
    confidence: float
    last_seen_age_s: float
```

`x_error` and `y_error` should describe the normalized offset of the target midpoint from the image center. `led_distance_px` is the pixel distance between the green and red LEDs when the pair is visible. `pair_angle_rad` is the image-plane angle from the green LED to the red LED.

When both LEDs are visible, the target point is their midpoint. When only one LED is visible, estimate the midpoint only if a recent green-to-red vector is available; otherwise report `visible=False` unless a later detector explicitly supports a safe short-term prediction mode.

## Implementation Files

- `drone_sdk/two_led_cv/__init__.py`: public exports for the perception contract.
- `drone_sdk/two_led_cv/types.py`: `LedBlob` and `TwoLedObservation` dataclasses.
- `drone_sdk/two_led_cv/protocol.py`: LED indexes, masks, states, `make_led_mask()`, `mask_for_state()`, `led_states_for_state()`, and `signal_on_for_state()`.
- Future detector module: `drone_sdk/two_led_cv/detector.py`.
- Integration point: `resources/scripts/follower.py` delegates to
  `henera_swarm.orchestration.follower_runtime`, which reads camera samples
  through `henera_swarm.perception.CVVisionProvider`.

## Manual Gazebo Checks

Start the swarm first, then publish masks to the leader model:

```bash
gz topic -t /model/x500_mono_cam_0/led_cmd -m gz.msgs.StringMsg -p 'data:"1100"'
```

Expected result: the green LED and red LED are both visible.

```bash
gz topic -t /model/x500_mono_cam_0/led_cmd -m gz.msgs.StringMsg -p 'data:"1000"'
```

Expected result: only the green LED is visible.

```bash
gz topic -t /model/x500_mono_cam_0/led_cmd -m gz.msgs.StringMsg -p 'data:"0100"'
```

Expected result: only the red LED is visible.
