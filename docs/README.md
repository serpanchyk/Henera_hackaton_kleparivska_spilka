# Falcon Gaze Documentation

Falcon Gaze is a Gazebo/PX4/ROS 2 starter package for a hackathon task: optical communication and leader-follower control for a simulated drone swarm.

The baseline scenario uses one leader drone and three follower drones in the `baylands_custom` Gazebo world. The leader may know the route and global state. Followers should solve the task using camera frames and visible LED signals, without reading the leader position, world ground truth, or direct digital messages from the leader.

## Documentation Map

- [Project Overview](overview.md): mission scope, repository layout, and runtime architecture.
- [Getting Started](getting-started.md): environment setup, PX4/Gazebo installation notes, project setup, and first launch.
- [Hackathon Rules](hackathon-rules.md): official task framing, allowed/prohibited data, LED budget, and submission format.
- [PYGR LED Protocol](pygr_led_protocol.md): four-LED fixed-color marker layout, masks, and Gazebo test commands.
- [SDK API](sdk-api.md): `drone_sdk` public API, ports, topics, camera, LED, and flight-control methods.
- [Examples](examples.md): current demo and swarm scripts.
- [Evaluation](evaluation.md): public/hidden scenarios, scoring rubric, metrics, penalties, and `results.json`.
- [Troubleshooting](troubleshooting.md): known setup and runtime issues.

## Source Material

These docs consolidate information from:

- `Хакатон_ГЕНЕРА_2_документація-1.pdf`
- `README.md`
- `README_setup.md`
- `SDK_AND_EXAMPLES.md`
- `resources/scripts/led_controller/README.md`
- Current repository code in `drone_sdk/`, `examples/`, `resources/scripts/`, and `project_setup.sh`
