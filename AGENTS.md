# AGENTS.md

## Project

Falcon Gaze is a Gazebo/PX4/ROS 2 starter repository for a simulated optical-communication drone-swarm hackathon. The central task is leader-follower control: drone 0 is the leader, follower drones use camera frames and LED signals rather than GPS, ground truth, or direct digital leader messages.

## Important Docs

Read these before changing behavior:

- `docs/README.md`
- `docs/overview.md`
- `docs/getting-started.md`
- `docs/hackathon-rules.md`
- `docs/sdk-api.md`
- `docs/examples.md`
- `docs/evaluation.md`
- `SDK_AND_EXAMPLES.md`
- `README_setup.md`

The official hackathon source document is `Хакатон_ГЕНЕРА_2_документація-1.pdf`.

## Repository Structure

- `drone_sdk/`: Python SDK wrapping MAVSDK plus ROS/Gazebo camera and LED bridges.
- `examples/`: runnable SDK examples for one drone and four-drone swarm control.
- `resources/scripts/`: PX4/Gazebo launch scripts and mission scripts.
- `resources/plugins/led_controller/`: Gazebo system plugin for LED visual commands.
- `resources/worlds/`: custom Gazebo world and media.
- `resources/x500_base/`, `resources/x500_mono_cam/`: model overrides copied into PX4.
- `project_setup.sh`: copies repository resources into `~/PX4-Autopilot/Tools/simulation/gz`.

## Environment Assumptions

The documented baseline is Windows 11 + WSL2 + Ubuntu 22.04, Gazebo Harmonic, PX4 `v1.15.4`, and ROS 2 Humble. Native Ubuntu 22.04 is also reasonable.

Most runtime commands require:

```bash
source /opt/ros/humble/setup.bash
```

Custom Gazebo resources require:

```bash
source ~/falcon_gaze/resources/scripts/px4_gz_setup.sh
```

Adjust `~/falcon_gaze` if the checkout path differs.

## Development Notes

- Prefer preserving the current simple SDK surface unless a task requires broader changes.
- Keep follower implementations within the hackathon constraints: camera and optical LED data only for leader-relative decisions.
- Do not add shortcuts that read leader/global pose for follower navigation unless clearly marked as organizer-only evaluation or debugging code.
- Keep docs in Ukrainian or English consistently within a file. Existing project docs are mostly Ukrainian; `docs/` currently uses English for agent and contributor clarity.
- Avoid committing generated runtime artifacts such as `__pycache__/`.

## Known Gaps

- The official PDF references `evaluate.py`, but no evaluator currently exists in the repo.
- The starter package provides examples and infrastructure, not a complete CV follower solution.
- Some script names and comments contain typos such as `swarn_launch.py`; preserve existing filenames unless intentionally migrating references.

## Common Commands

Copy resources into PX4:

```bash
./project_setup.sh
```

Launch swarm:

```bash
cd ~/PX4-Autopilot
ros2 launch ~/falcon_gaze/resources/scripts/swarn_launch.py
```

Run leader mission:

```bash
cd ~/falcon_gaze/resources/scripts
python3 mission_launch.py
```

Run SDK demo:

```bash
python3 examples/demo.py
```

