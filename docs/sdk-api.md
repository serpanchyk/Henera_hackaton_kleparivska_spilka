# SDK API

The Python SDK in `drone_sdk/` wraps two subsystems:

- MAVSDK for flight control, offboard commands, and telemetry.
- ROS 2 + Gazebo bridges for camera frames and LED commands.

Import from the package root:

```python
from drone_sdk import Drone, MAVSDKError
```

## Ports

MAVSDK ports are derived from `drone_id` and the base ports in `config.yaml`
(`ports.mavsdk_udp_base`, `ports.mavsdk_grpc_base`). With the default config:

| Drone | UDP | gRPC |
| --- | ---: | ---: |
| 0 | 14540 | 50051 |
| 1 | 14541 | 50052 |
| 2 | 14542 | 50053 |
| 3 | 14543 | 50054 |

The SDK computes:

```text
UDP = CONFIG.ports.mavsdk_udp_base + drone_id
gRPC = CONFIG.ports.mavsdk_grpc_base + drone_id
```

## Gazebo / ROS Topics

Camera:

```text
/world/baylands_custom/model/x500_mono_cam_{id}/link/mono_cam/base_link/sensor/camera_sensor/image
```

LED:

```text
/model/x500_mono_cam_{id}/led_cmd
```

## Connection

```python
drone = Drone(drone_id=0)
await drone.connect(timeout=20.0)
```

`connect()` waits for:

- MAVSDK connection state;
- PX4 global position health;
- PX4 home position health.

Errors:

- `TimeoutError`: connection call exceeded timeout.
- `ConnectionError`: connection or health state did not become ready.

## Basic Flight Actions

```python
await drone.arm()
await drone.disarm()
await drone.takeoff(altitude_m=10.0)
await drone.land()
await drone.set_takeoff_altitude(altitude_m)
```

These methods require a successful `connect()` first.

## Offboard Control

Start offboard mode:

```python
await drone.start_offboard()
```

The method sends several initial position setpoints before starting offboard mode, because PX4 requires a setpoint stream before offboard activation.

Stop offboard:

```python
await drone.stop_offboard()
```

## Position Commands

```python
await drone.go_to(north, east, down, yaw_deg=0.0, body_frame=False)
```

With `body_frame=False`, `north`, `east`, and `down` are absolute NED coordinates.

With `body_frame=True`, they are offsets relative to the current drone heading:

- `north`: forward offset;
- `east`: right offset;
- `down`: vertical offset;
- current heading is preserved as yaw.

`go_to()` only sends a setpoint. It does not wait until the drone reaches the target.

## Velocity Commands

Body-relative velocity:

```python
await drone.move(forward, right, down, speed_m_s=2.0)
```

The vector is rotated into global NED using the current heading and normalized to `speed_m_s`.

Global NED velocity:

```python
await drone.set_velocity(north_m_s, east_m_s, down_m_s, yaw_deg=None)
```

Use this when your controller already computes commands in the world/NED frame.

## Telemetry

```python
pos = await drone.position_ned()
heading = await drone.heading()
```

`position_ned()` returns `PositionNED(north_m, east_m, down_m)`.

## LED Control

LED methods lazily initialize ROS and bridge subprocesses:

```python
drone.set_leds("1000")
drone.led_on()
drone.led_off()
drone.led_blink()
```

Supported command forms in the current Gazebo plugin:

- four-character binary masks: `1100`, `1000`, etc.;
- `ON`;
- `OFF`;
- `BLINK`.

For the current two-lens model, mask bit 1 controls `led_lens_01` as the fixed green LED and mask bit 2 controls `led_lens_04` as the fixed red LED. Bits 3 and 4 are unused compatibility padding.

Protocol masks:

- `FOLLOW`: green + red ON, `1100`.
- `HOLD`: green ON only, `1000`.
- `SAFE`: red ON only, `0100`.
- `FINISH`: green + red blink together in phase, alternating `1100` and `0000` at about 1 Hz.

## Camera

Inline spin pattern:

```python
drone.start_camera()

while running:
    drone.spin()
    frame = drone.camera_frame()
    if frame is not None:
        ...
```

Methods:

- `start_camera()`: starts bridges and creates the ROS node, but does not start a background spin thread.
- `spin()`: processes one ROS callback cycle.
- `camera_frame()`: returns the latest cached OpenCV BGR frame or `None`.
- `stop_camera()`: stops node spinning and bridge subprocesses.

## Cleanup

```python
await drone.close()
```

`close()` stops camera and bridge resources and resets SDK connection state. It does not call `rclpy.shutdown()` for the whole process; examples do that at top level.

## Exceptions

Exported exception classes:

- `DroneSDKError`
- `ConnectionError`
- `TimeoutError`
- `MAVSDKError`
- `GazeboError`
- `CameraError`
- `LEDError`

The current SDK actively raises `ConnectionError`, `TimeoutError`, and `MAVSDKError`. Other classes are available for consistent caller-side handling.
