#!/usr/bin/env python3
"""
DEBUG / EVAL ONLY — ground-truth cheat, NEVER SUBMIT.

This provider fabricates a `VisualObservation` for the follower controller by
reading the GROUND-TRUTH relative position between two drones (via MAVSDK local
telemetry), instead of detecting the leader's LED in a camera frame.

It exists solely to test the follower CONTROLLER in Gazebo before the real
camera-based CV pipeline (Person 2) is ready — "test control independently of
perception". It reads the target/leader position, which is a FORBIDDEN topic
for the hackathon submission. CLAUDE.md allows this only when clearly marked as
debug/eval and kept out of the final solution. Do not wire this into follower.py.

Geometry
--------
Each PX4 instance's local NED origin = its spawn point (EKF home), with axes
aligned to true North/East/Down regardless of spawn yaw. So:
    world_E = spawn_x + east_m
    world_N = spawn_y + north_m
    world_U = spawn_z - down_m
Then, for follower F (heading h deg from North, clockwise) and target T:
    forward =  dN*cos(hr) + dE*sin(hr)
    right   = -dN*sin(hr) + dE*cos(hr)
    up      =  dU
    horizontal_angle_deg = atan2(right, forward)      # +right  -> +yaw toward target
    vertical_angle_deg   = atan2(-up, horiz_dist)      # target below -> +down command
    target_size          = SIZE_GAIN / distance_3d     # closer -> bigger

Sign conventions (match FollowerController):
    yaw_sign = +1.0      -> target on the right yaws clockwise toward it
    vertical_sign = +1.0 -> target below commands positive down
If the first sim run shows a follower yawing AWAY from its target, flip
`yaw_sign` to -1.0 in FollowerControllerConfig (do NOT change the math here).
If a follower climbs when it should descend, flip `vertical_sign` to -1.0.
"""
import math
import time

from drone_sdk.follower_controller import MissionState, VisualObservation


class DebugVisionProvider:
    """Produces ground-truth VisualObservation for one follower. DEBUG ONLY."""

    def __init__(
        self,
        follower_drone,
        target_drone,
        leader_drone,
        follower_spawn,
        target_spawn,
        leader_spawn,
        size_gain: float = 240.0,
        desired_distance_m: float = 3.0,
        min_dist_m: float = 0.3,
        finish_altitude_m: float = 1.0,
        climb_latch_m: float = 3.0,
    ):
        self.follower_drone = follower_drone
        self.target_drone = target_drone
        self.leader_drone = leader_drone
        self.follower_spawn = follower_spawn  # (x_east, y_north, z_up)
        self.target_spawn = target_spawn
        self.leader_spawn = leader_spawn
        self.size_gain = size_gain
        self.desired_distance_m = desired_distance_m
        self.min_dist_m = min_dist_m
        self.finish_altitude_m = finish_altitude_m
        self.climb_latch_m = climb_latch_m
        self._climbed = False  # latch: only allow FINISH after the leader has flown

    @staticmethod
    def _to_world(pos_ned, spawn):
        """Local NED (north_m, east_m, down_m) + spawn (E, N, U) -> world (E, N, U)."""
        world_e = spawn[0] + pos_ned.east_m
        world_n = spawn[1] + pos_ned.north_m
        world_u = spawn[2] - pos_ned.down_m
        return world_e, world_n, world_u

    async def _mission_state(self) -> MissionState:
        """FOLLOW while the leader flies; FINISH once it has flown and then landed."""
        leader_ned = await self.leader_drone.position_ned()
        _, _, leader_u = self._to_world(leader_ned, self.leader_spawn)
        if leader_u > self.climb_latch_m:
            self._climbed = True
        if self._climbed and leader_u <= self.finish_altitude_m:
            return MissionState.FINISH
        return MissionState.FOLLOW

    async def observe(self) -> VisualObservation:
        target_ned = await self.target_drone.position_ned()
        follower_ned = await self.follower_drone.position_ned()
        heading_deg = await self.follower_drone.heading()

        t_e, t_n, t_u = self._to_world(target_ned, self.target_spawn)
        f_e, f_n, f_u = self._to_world(follower_ned, self.follower_spawn)

        d_e = t_e - f_e
        d_n = t_n - f_n
        d_u = t_u - f_u

        hr = math.radians(heading_deg)
        forward = d_n * math.cos(hr) + d_e * math.sin(hr)
        right = -d_n * math.sin(hr) + d_e * math.cos(hr)
        up = d_u

        horizontal_distance = math.hypot(forward, right)
        distance_3d = math.sqrt(forward * forward + right * right + up * up)

        horizontal_angle_deg = math.degrees(math.atan2(right, forward))
        vertical_angle_deg = math.degrees(math.atan2(-up, horizontal_distance))
        target_size = self.size_gain / max(distance_3d, self.min_dist_m)

        mission_state = await self._mission_state()

        return VisualObservation(
            target_visible=True,
            horizontal_angle_deg=horizontal_angle_deg,
            vertical_angle_deg=vertical_angle_deg,
            target_size=target_size,
            mission_state=mission_state,
            timestamp=time.monotonic(),
        )

    async def __call__(self) -> VisualObservation:
        return await self.observe()
