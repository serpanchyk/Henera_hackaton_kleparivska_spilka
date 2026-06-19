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

- `1000`
- `0100`
- `0010`
- `0001`
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

Leader-only MAVSDK mission script. It connects to drone 0, takes off, captures current heading/altitude, starts offboard mode, runs a simple path/training pattern, and lands.

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
        {"mask": "1010", "ticks": 4},
        {"mask": "0000", "ticks": 4},
        {"mask": "0101", "ticks": 2}
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

