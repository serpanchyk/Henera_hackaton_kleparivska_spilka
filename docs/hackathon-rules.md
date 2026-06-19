# Hackathon Rules

## Event Format

- Date: 19-20 June 2026.
- Format: two-day departmental hackathon.
- Teams: up to 10 teams, 3-4 participants per team.
- Simulator: Gazebo with PX4 SITL and starter code supplied by organizers.

## Official Task

In Gazebo, there is one leader drone and several follower drones. The leader knows its route and global position. Followers do not have GPS and must not read the leader position, world ground truth, or direct leader messages. Followers must move behind the leader using only camera input and optical LED/visual beacon signals.

The team must implement optical communication, signal decoding, and follower control to keep formation and complete the mission.

## Minimum Successful Result

A baseline successful solution should:

- launch in the supplied simulator;
- use one leader and at least two follower drones;
- make the leader transmit visible LED or marker patterns;
- detect and decode the leader signal from follower camera frames;
- move followers through the route without forbidden topics;
- avoid critical collisions and crashes;
- log metrics to `results.json` or `.csv`;
- be reproducible from the submitted README.

## Recommended MVP

The organizer MVP is:

- 3 drones total: one leader and two followers;
- daytime scene with stable lighting;
- one route with several checkpoints;
- leader LED heartbeat and simple state or direction code;
- follower safe-hover behavior on signal loss;
- public test scenarios before final hidden tests.

## Allowed Data

Followers may use:

- their own camera frames;
- decoded optical/LED messages visible in the camera;
- their own IMU/local telemetry required for stabilization and control;
- their own velocity, attitude, altitude, and local state exposed through the starter API;
- public scenario configuration that organizers explicitly allow.

## Prohibited Data

Followers must not use:

- global GPS or world coordinates for navigation;
- leader ground-truth position or pose;
- direct ROS/Gazebo/PX4 topics that expose the leader state;
- simulator internals that reveal target/checkpoint truth unless explicitly allowed;
- direct digital communication from leader to followers outside the optical channel.

## Topic Policy

Solutions should clearly list every topic/service/API they read and write. Organizer-side checking should verify topic usage during final runs. Reading prohibited topics can result in zero score for the scenario or disqualification.

## LED Budget

The official task should keep the optical channel limited and comparable across teams. The source PDF recommends a fixed LED budget instead of letting teams add arbitrary beacons.

Suggested baseline:

- one LED group on the leader;
- visible camera-based detection from followers;
- limited colors/channels and blink frequencies;
- no unlimited auxiliary markers that turn the task into direct localization.

This repository currently models two LED lenses per drone. The Gazebo plugin still accepts four-character binary masks for SDK compatibility: bit 1 controls the green anchor lens, bit 2 controls the red signal lens, and bits 3-4 are unused by the current model. It also accepts `ON`, `OFF`, and `BLINK` legacy commands.

## Minimum Protocol Requirements

The optical protocol should define:

- message/heartbeat pattern;
- timing or frame duration;
- at least one leader state or route/following signal;
- timeout behavior when a message is not decoded;
- basic error handling for missing or ambiguous frames.

## Expected Follower Logic

A follower should:

1. Read camera frames.
2. Detect LED or marker candidates.
3. Estimate relative bearing, approximate scale, and signal state.
4. Decode the optical message.
5. Generate motion commands for centering, distance keeping, and route following.
6. Enter safe hover or slow search after a timeout.
7. Reacquire the signal and continue when possible.

## Submission

Each team submits:

- repository or archive with code;
- README with build/run commands, dependencies, protocol description, and topic usage;
- `results.json` from public tests;
- short demo video or screen recording if live demo fails;
- 4-5 slides covering architecture, protocol, CV pipeline, results, and conclusions.

## Final Presentation

Recommended timing:

- Demo: 4 minutes.
- Architecture: 3 minutes.
- Protocol: 3 minutes.
- Results: 3 minutes.
- Q&A: 2 minutes.

