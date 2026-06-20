# Falcon Gaze Runtime Architecture

## Source Of Truth

`drone_sdk/` is the public SDK package and the compatibility contract inherited
from the starter repository. Reusable flight, ROS/Gazebo bridge, camera, LED,
controller, startup, and two-LED CV modules live there.

`examples/drone_sdk/` is kept only as a compatibility package for older example
launches that put `examples/` first on `sys.path`. Do not add new implementation
logic there.

## Runtime Package

`resources/scripts/henera_swarm/` contains the internal runtime package used by
the launch scripts:

- `perception`: camera providers that convert frames and LED state into
  `VisualObservation` values.
- `orchestration`: executable mission/follower runtime loops.
- `logging`: hackathon metric logging helpers.
- `launch_utils`: shared PX4 spawn and formation helpers.

## Control Flow

The official final demo path is:

```text
resources/scripts/solution_launch.py
  -> starts PX4 leader and followers in train formation
  -> starts resources/scripts/mission_launch.py for the leader
  -> starts resources/scripts/follower.py for drones 1, 2, and 3
  -> henera_swarm.orchestration.follower_runtime
```

`follower.py` is intentionally a thin entrypoint. The production follower loop
composes:

```text
Drone
CVVisionProvider
FollowerController
DroneFollowerActuator
ResultsLogger
LED relay output
```

`resources/scripts/cv_launch.py` and `cv_swarm_test.py` remain useful for the
single-process CV development path, but they are not the official final path.

## Debug Code

`debug_vision_provider.py` reads ground-truth relative position and is forbidden
for final hackathon navigation. Keep it out of `solution_launch.py` and
`follower.py`.
