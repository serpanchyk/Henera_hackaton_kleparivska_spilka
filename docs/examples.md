# Examples

Run examples after the Gazebo/PX4 swarm is already running and ROS 2 is sourced:

```bash
source /opt/ros/humble/setup.bash
```

## `examples/demo.py`

Single-drone SDK demonstration.

It shows:

- connecting to drone 0;
- showing camera frames in an OpenCV window;
- running an LED mask animation;
- arming, takeoff, and offboard start;
- body-relative movement with `go_to()`;
- landing, disarm, and cleanup.

Run:

```bash
python3 examples/demo.py
```

Movement sequence:

1. Take off to 10 m.
2. Start offboard mode.
3. Move 10 m forward.
4. Move 10 m right.
5. Return with `go_to(-10, -10, 0, body_frame=True)`.

LED pattern:

- `1111`
- `1100`
- `BLINK`

## `examples/swarm_velocities.py`

Four-drone swarm example using continuous velocity control.

Key constants:

- `SWARM_SIZE = 4`
- `ALTITUDE = 10.0`
- `SPEED = 2.0`
- `FLIGHT_DURATION = 40`

Behavior:

- connects to all four drones;
- opens one camera window per drone;
- runs LED animation on drone 0;
- arms and takes off all drones;
- starts offboard for each drone;
- repeatedly sends forward body-frame velocity commands with `move()`;
- stops movement, stops offboard, lands, and disarms.

Run:

```bash
python3 examples/swarm_velocities.py
```

## `examples/swarm_waypoints.py`

Four-drone swarm example using body-relative waypoint setpoints.

Waypoint sequence:

1. `go_to(10.0, 0.0, 0.0, body_frame=True)`
2. `go_to(10.0, -10.0, 0.0, body_frame=True)`
3. `go_to(10.0, 10.0, 0.0, body_frame=True)`
4. `go_to(10.0, 0.0, 0.0, body_frame=True)`

In body-relative interpretation this means:

1. 10 m forward.
2. 10 m forward and 10 m left.
3. 10 m forward and 10 m right.
4. 10 m forward.

Run:

```bash
python3 examples/swarm_waypoints.py
```

## `resources/scripts/mission_launch.py`

Leader-only MAVSDK mission script. It connects to drone 0, takes off, captures current heading/altitude, starts offboard mode, flies straight forward at constant altitude, briefly hovers, and lands.

Run after swarm launch:

```bash
cd ~/falcon_gaze/resources/scripts
python3 mission_launch.py
```

## LED Controller Script

The LED controller helper in `resources/scripts/led_controller/` can replay JSON-defined LED timelines.

Example pattern file:

```json
{
  "swarm_missions": {
    "my_custom_point": {
      "delay_time": 0.25,
      "loop": true,
      "timeline": [
        {"mask": "1111", "ticks": 4},
        {"mask": "0000", "ticks": 4},
        {"mask": "1100", "ticks": 2}
      ]
    }
  }
}
```

Launch form:

```bash
source /opt/ros/humble/setup.bash
cd ~/falcon_gaze/resources/scripts/led_controller
ros2 launch led_launch.py id:=0 file:=blink.json point:=my_custom_point
```

Arguments:

- `id:=0`: target drone index.
- `file:=blink.json`: JSON pattern file in the script directory.
- `point:=my_custom_point`: named pattern section in the JSON file.

## Two-LED CV Debug Runner

`resources/scripts/two_led_cv_debug.py` runs the perception-only two-LED pipeline:

This is legacy/debug code for the earlier two-LED experiment and is not the PYGR marker decoder.

1. read camera, webcam, video, or image frames;
2. detect the green anchor and red signal LEDs;
3. update `TwoLedCommandDecoder`;
4. update `TwoLedTracker`;
5. draw a debug overlay and print throttled observations.

It does not connect to MAVSDK, publish flight commands, or move drones.

Start the simulation first:

```bash
cd ~/PX4-Autopilot
source ~/falcon_gaze/resources/scripts/px4_gz_setup.sh
source /opt/ros/humble/setup.bash
ros2 launch ~/falcon_gaze/resources/scripts/swarn_launch.py
```

In another ROS-sourced terminal, list camera topics:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep camera_sensor
```

If the camera topic is not visible in ROS, bridge it from Gazebo:

```bash
ros2 run ros_gz_image image_bridge \
  /world/baylands_custom/model/x500_mono_cam_1/link/mono_cam/base_link/sensor/camera_sensor/image
```

Run the debug script against a follower camera topic:

```bash
python3 resources/scripts/two_led_cv_debug.py \
  --source ros:/world/baylands_custom/model/x500_mono_cam_1/link/mono_cam/base_link/sensor/camera_sensor/image \
  --debug \
  --show-mask
```

Offline fallback modes:

```bash
python3 resources/scripts/two_led_cv_debug.py --source 0 --debug
python3 resources/scripts/two_led_cv_debug.py --source ./sample_video.mp4 --debug
python3 resources/scripts/two_led_cv_debug.py --source ./sample_frame.png --debug --show-mask
```

Test the active PYGR leader LED masks directly with Gazebo:

```bash
gz topic -t /model/x500_mono_cam_0/led_cmd -m gz.msgs.StringMsg -p 'data:"1111"'
```

Expected result: purple, yellow, green, and red LEDs are visible.

```bash
gz topic -t /model/x500_mono_cam_0/led_cmd -m gz.msgs.StringMsg -p 'data:"1100"'
```

Expected result: only the purple target and yellow distance LEDs are visible.

The overlay shows detected contours, LED blob centers, selected pair line, midpoint or estimated midpoint, `x_error`, `y_error`, `led_distance_px`, decoded `state`, `signal_on_ratio`, and `transitions_per_s`.
