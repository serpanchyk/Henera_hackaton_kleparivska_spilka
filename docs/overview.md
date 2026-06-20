# Project Overview

## Goal

The project supports a two-day departmental hackathon focused on optical/visual communication in a simulated drone swarm. Teams build:

- a computer-vision module for detecting LED beacons or visual markers;
- an optical message encoder/decoder;
- a follower-control algorithm that keeps drones behind a leader or another drone in a chain;
- formation and distance-keeping logic;
- timeout, safe-hover, search, and reacquisition behavior when signal is lost;
- metric logging for evaluation.

The base task is intentionally scoped as a leader-follower problem. Full mesh communication, reserve leader election, hard lighting, and relay between followers are bonus directions, not baseline requirements.

## Runtime Stack

- Gazebo Harmonic: simulation world, rendering, model visuals, and LED plugin execution.
- PX4 SITL: drone autopilot and stabilization.
- ROS 2 Humble: launch, topics, camera bridges, and LED command publishing.
- MAVSDK Python: drone connection, arming, takeoff, landing, telemetry, and offboard commands.
- OpenCV/cv_bridge: camera frame handling in Python examples and SDK consumers.

## Repository Layout

```text
.
├── drone_sdk/                  # Python SDK for MAVSDK + ROS/Gazebo camera and LED access
├── examples/                   # Single-drone and swarm usage examples
├── resources/
│   ├── plugins/led_controller/ # Gazebo system plugin for LED visual control
│   ├── scripts/                # PX4/Gazebo launch and mission scripts
│   ├── worlds/                 # baylands_custom Gazebo world and media
│   ├── x500_base/              # Modified base model
│   └── x500_mono_cam/          # Drone model with camera and LED visuals
├── project_setup.sh            # Copies resources into a local PX4 checkout
├── README.md                   # Short run notes
├── README_setup.md             # Environment setup notes
└── SDK_AND_EXAMPLES.md         # SDK behavior and example script notes
```

## Simulation Shape

`resources/scripts/swarn_launch.py` launches four PX4 SITL instances:

- Drone 0: leader, also starts Gazebo.
- Drones 1, 2, 3: followers, launched after a short delay so Gazebo is ready.

All drones use the `gz_x500_mono_cam` model in the `baylands_custom` world. Each drone has a camera and two fixed-color LED lenses controlled through a Gazebo topic: mask bit 1 drives `led_lens_01` as green, mask bit 2 drives `led_lens_04` as red, and bits 3-4 are unused. The protocol uses green+red for `FOLLOW`, green only for `HOLD`, red only for `SAFE`, and synchronized green+red blinking for `FINISH`.

## Main Data Flow

1. PX4 SITL exposes one MAVSDK UDP endpoint per drone.
2. `drone_sdk.Drone.connect()` connects to the matching endpoint.
3. Camera and LED access are initialized lazily when camera/LED methods are called.
4. `BridgeManager` starts ROS/Gazebo bridge subprocesses.
5. `DroneROSNode` subscribes to camera frames and publishes LED commands.
6. User code repeatedly calls `drone.spin()` when it needs fresh camera frames.

## Current Scope

The repository provides a starter package, examples, modified Gazebo assets, and API wrappers. It does not yet include a complete follower CV/control solution or an `evaluate.py` implementation.
