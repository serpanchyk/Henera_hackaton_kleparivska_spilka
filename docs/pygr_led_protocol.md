# PYGR LED Protocol

The Gazebo drone model uses a fixed-color four-LED rear marker for optical leader-follower communication. The LEDs are controlled through the existing binary mask topic:

```text
/model/<MODEL_NAME>/led_cmd
```

Runtime commands only turn each LED ON or OFF. LED colors are fixed by role and do not change during flight.

## Marker Layout

The visual names and mask order are:

| Visual name | Mask bit | Role | Color |
| --- | --- | --- | --- |
| `led_lens_01` | `mask[0]` | P / tracking target | purple / magenta |
| `led_lens_02` | `mask[1]` | Y / distance reference | yellow |
| `led_lens_03` | `mask[2]` | G / command bit | green |
| `led_lens_04` | `mask[3]` | R / command bit | red |

`led_lens_01` and `led_lens_02` form the stable vertical baseline for distance estimation. `led_lens_01` is the upper tracking target. `led_lens_03` and `led_lens_04` are command bits.

## State Masks

| State | P | Y | G | R | Mask |
| --- | --- | --- | --- | --- | --- |
| `FOLLOW` | ON | ON | ON | ON | `1111` |
| `HOLD` | ON | ON | ON | OFF | `1110` |
| `FINISH` | ON | ON | OFF | ON | `1101` |
| `SAFE` | ON | ON | OFF | OFF | `1100` |

No blinking or frequency decoding is required for these protocol states.

## Macro Commands

- `ON`: turns all four LEDs on with their fixed PYGR colors.
- `OFF`: turns all four LEDs off.
- `BLINK`: debug helper that blinks all four fixed-color LEDs on and off together.

## Gazebo Test Commands

Replace `<MODEL_NAME>` with a model such as `x500_mono_cam_0`.

```bash
gz topic -t /model/<MODEL_NAME>/led_cmd \
  -m gz.msgs.StringMsg \
  -p 'data: "1111"'
```

```bash
gz topic -t /model/<MODEL_NAME>/led_cmd \
  -m gz.msgs.StringMsg \
  -p 'data: "1110"'
```

```bash
gz topic -t /model/<MODEL_NAME>/led_cmd \
  -m gz.msgs.StringMsg \
  -p 'data: "1101"'
```

```bash
gz topic -t /model/<MODEL_NAME>/led_cmd \
  -m gz.msgs.StringMsg \
  -p 'data: "1100"'
```
