# Two-LED CV Decoder Plan

## Confirmed Architecture

- LED commands are sent to `/model/x500_mono_cam_{id}/led_cmd`.
- Direct Gazebo commands use `gz.msgs.StringMsg`.
- ROS-side commands use `std_msgs/msg/String` bridged to `gz.msgs.StringMsg`.
- Camera frames are published at `/world/baylands_custom/model/x500_mono_cam_{id}/link/mono_cam/base_link/sensor/camera_sensor/image`.
- The camera model is `640x480`, `30 FPS`, with horizontal FOV `1.6` radians.

The Gazebo LED plugin finds visuals named `led_lens_*`, sorts them by name, and applies a four-character binary mask. The current model defines `led_lens_01` and `led_lens_04`. The active two-lens protocol uses bit 1 for the anchor and bit 2 for the signal; bits 3 and 4 are unused:

| Mask | Meaning |
| --- | --- |
| `1100` | green anchor ON, red signal ON |
| `1000` | green anchor ON, red signal OFF |

LED color is fixed by lens role: `led_lens_01` is green when enabled, `led_lens_04` is red when enabled, and disabled LEDs are dark.

## LED Protocol

- `led_lens_01` is the green anchor LED and should remain ON during normal operation.
- `led_lens_04` is the red signal LED and encodes command state by blinking.
- `FOLLOW`: red signal always ON, mask `1100`.
- `HOLD`: red signal toggles every `0.5` seconds.
- `FINISH`: red signal toggles every `0.2` seconds.
- `SAFE` or unknown state: red signal always OFF, mask `1000`.

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

`x_error` and `y_error` should describe the normalized offset of the target midpoint from the image center. `led_distance_px` is the pixel distance between the green anchor and red signal when the pair is visible. `pair_angle_rad` is the image-plane angle from anchor LED to signal LED.

When both LEDs are visible, the target point is their midpoint. When the red signal is OFF, estimate the midpoint from the green anchor plus the last known anchor-to-signal vector. If the green anchor is not visible, report `visible=False` unless a later detector explicitly supports a safe short-term prediction mode.

## Implementation Files

- `drone_sdk/two_led_cv/__init__.py`: public exports for the perception contract.
- `drone_sdk/two_led_cv/types.py`: `LedBlob` and `TwoLedObservation` dataclasses.
- `drone_sdk/two_led_cv/protocol.py`: LED indexes, masks, states, `make_led_mask()`, and `signal_on_for_state()`.
- Future detector module: `drone_sdk/two_led_cv/detector.py`.
- Future integration point: replace `detect_led()` in `resources/scripts/follower.py` with a wrapper around the detector.

## Manual Gazebo Checks

Start the swarm first, then publish masks to the leader model:

```bash
gz topic -t /model/x500_mono_cam_0/led_cmd -m gz.msgs.StringMsg -p 'data:"1100"'
```

Expected result: the anchor is green and the signal is red.

```bash
gz topic -t /model/x500_mono_cam_0/led_cmd -m gz.msgs.StringMsg -p 'data:"1000"'
```

Expected result: the anchor remains green and the red signal turns off.
