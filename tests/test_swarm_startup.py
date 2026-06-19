import asyncio
import time
import unittest

from drone_sdk.follower_controller import (
    FollowerControllerConfig,
    MissionState,
    VisualObservation,
)
from drone_sdk.swarm_startup import (
    AlignmentStatus,
    StartupConfig,
    StartupError,
    alignment_command,
    align_chain_sequentially,
    align_follower_to_target,
    is_alignment_ready,
    prepare_swarm_for_start,
    safe_stop_all,
    wait_for_all_ready_then_start,
)


class MockPosition:

    def __init__(self, down_m):
        self.down_m = down_m


class MockDrone:

    def __init__(self, drone_id=1, height_m=10.0, fail_takeoff=False):
        self.drone_id = drone_id
        self.height_m = height_m
        self.fail_takeoff = fail_takeoff
        self.connected = True
        self.armed = False
        self.takeoff_altitudes = []
        self.offboard_started = False
        self.velocity_commands = []
        self.heading_deg = 0.0

    async def arm(self):
        self.armed = True

    async def takeoff(self, altitude_m=10.0):
        if self.fail_takeoff:
            raise RuntimeError('takeoff failed')
        self.takeoff_altitudes.append(altitude_m)

    async def position_ned(self):
        return MockPosition(-self.height_m)

    async def heading(self):
        return self.heading_deg

    async def set_velocity(self, north_m_s, east_m_s, down_m_s, yaw_deg=None):
        self.velocity_commands.append((north_m_s, east_m_s, down_m_s, yaw_deg))

    async def start_offboard(self):
        self.offboard_started = True


def obs(
    visible=True,
    h=0.0,
    v=0.0,
    size=80.0,
    state=MissionState.FOLLOW,
    timestamp=None,
):
    return VisualObservation(
        target_visible=visible,
        horizontal_angle_deg=h,
        vertical_angle_deg=v,
        target_size=size,
        mission_state=state,
        timestamp=time.monotonic() if timestamp is None else timestamp,
    )


def startup_config(**overrides):
    values = dict(
        startup_altitude_m=10.0,
        startup_timeout_s=0.03,
        startup_settle_time_s=0.0,
        startup_height_tolerance_m=0.2,
        startup_poll_interval_s=0.001,
        alignment_timeout_s=0.2,
        alignment_required_frames=2,
        alignment_horizontal_tolerance_deg=1.0,
        alignment_vertical_tolerance_deg=1.0,
        alignment_size_tolerance=2.0,
        alignment_max_forward_speed=0.3,
        alignment_max_yaw_rate=5.0,
        alignment_max_vertical_speed=0.2,
        alignment_observation_timeout_s=0.2,
        alignment_search_yaw_rate=3.0,
    )
    values.update(overrides)
    return StartupConfig(**values)


class SwarmStartupTests(unittest.IsolatedAsyncioTestCase):

    async def test_synchronized_takeoff_runs_for_all_drones(self):
        leader = MockDrone(drone_id=0)
        followers = [MockDrone(drone_id=1), MockDrone(drone_id=2)]
        statuses = await prepare_swarm_for_start(leader, followers, startup_config())
        self.assertTrue(all(status.ready for status in statuses))
        self.assertEqual(leader.takeoff_altitudes, [10.0])
        self.assertEqual(followers[0].takeoff_altitudes, [10.0])
        self.assertEqual(followers[1].takeoff_altitudes, [10.0])
        self.assertTrue(all(drone.offboard_started for drone in [leader, *followers]))

    async def test_failed_takeoff_blocks_mission_and_safe_stops_all(self):
        leader = MockDrone(drone_id=0)
        followers = [MockDrone(drone_id=1, fail_takeoff=True), MockDrone(drone_id=2)]
        with self.assertRaises(StartupError):
            await prepare_swarm_for_start(leader, followers, startup_config())
        self.assertTrue(all(drone.velocity_commands for drone in [leader, *followers]))

    async def test_startup_timeout_safe_stops_all(self):
        leader = MockDrone(drone_id=0, height_m=0.0)
        followers = [MockDrone(drone_id=1)]
        with self.assertRaises(StartupError):
            await prepare_swarm_for_start(leader, followers, startup_config())
        self.assertTrue(leader.velocity_commands)
        self.assertTrue(followers[0].velocity_commands)

    async def test_safe_stop_all_sends_zero_velocity(self):
        drones = [MockDrone(drone_id=0), MockDrone(drone_id=1)]
        await safe_stop_all(drones)
        self.assertEqual(drones[0].velocity_commands[-1], (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(drones[1].velocity_commands[-1], (0.0, 0.0, 0.0, 0.0))

    async def test_leader_mission_waits_for_all_ready(self):
        all_ready_event = asyncio.Event()
        started = []

        async def leader_mission():
            started.append(True)

        task = asyncio.create_task(
            wait_for_all_ready_then_start(all_ready_event, leader_mission)
        )
        await asyncio.sleep(0)
        self.assertEqual(started, [])
        all_ready_event.set()
        await task
        self.assertEqual(started, [True])

    async def test_chain_alignment_is_sequential(self):
        followers = [MockDrone(drone_id=1), MockDrone(drone_id=2)]
        order = []

        def provider_1():
            order.append('follower_1')
            return obs()

        def provider_2():
            order.append('follower_2')
            return obs()

        results = await align_chain_sequentially(
            followers,
            [provider_1, provider_2],
            startup_config(),
            FollowerControllerConfig.stable(),
        )
        self.assertEqual([result.status for result in results], [AlignmentStatus.READY] * 2)
        self.assertLess(order.index('follower_1'), order.index('follower_2'))

    async def test_alignment_ready_requires_consecutive_frames(self):
        follower = MockDrone(drone_id=1)
        frames = [5.0, 0.0, 0.0]

        def provider():
            return obs(h=frames.pop(0)) if frames else obs()

        result = await align_follower_to_target(
            follower,
            provider,
            'follower_1',
            'leader',
            startup_config(alignment_required_frames=2),
            FollowerControllerConfig.stable(),
        )
        self.assertEqual(result.status, AlignmentStatus.READY)
        self.assertGreaterEqual(result.ready_frames, 2)

    async def test_alignment_timeout_fails_and_safe_stops(self):
        follower = MockDrone(drone_id=1)

        def provider():
            return obs(visible=False)

        result = await align_follower_to_target(
            follower,
            provider,
            'follower_1',
            'leader',
            startup_config(alignment_timeout_s=0.005),
            FollowerControllerConfig.stable(),
        )
        self.assertEqual(result.status, AlignmentStatus.FAILED)
        self.assertTrue(follower.velocity_commands)


class AlignmentLogicTests(unittest.TestCase):

    def test_ready_tolerances(self):
        cfg = startup_config()
        self.assertTrue(is_alignment_ready(obs(), cfg, current_time=time.monotonic()))
        self.assertFalse(is_alignment_ready(obs(h=2.0), cfg, current_time=time.monotonic()))
        self.assertFalse(is_alignment_ready(obs(v=2.0), cfg, current_time=time.monotonic()))
        self.assertFalse(is_alignment_ready(obs(size=70.0), cfg, current_time=time.monotonic()))

    def test_target_loss_blocks_forward_and_vertical(self):
        command = alignment_command(obs(visible=False), startup_config())
        self.assertEqual(command.forward_m_s, 0.0)
        self.assertEqual(command.down_m_s, 0.0)
        self.assertEqual(command.right_m_s, 0.0)
        self.assertGreater(command.yaw_rate_deg_s, 0.0)
        self.assertEqual(command.relay_state, MissionState.HOLD)

    def test_alignment_command_uses_limited_motion(self):
        command = alignment_command(
            obs(h=100.0, v=100.0, size=0.0),
            startup_config(),
            FollowerControllerConfig.responsive(),
        )
        self.assertLessEqual(abs(command.yaw_rate_deg_s), 5.0)
        self.assertLessEqual(abs(command.down_m_s), 0.2)
        self.assertLessEqual(abs(command.forward_m_s), 0.3)
        self.assertEqual(command.relay_state, MissionState.HOLD)

    def test_stale_or_hold_observation_is_not_ready(self):
        cfg = startup_config()
        now = time.monotonic()
        self.assertFalse(
            is_alignment_ready(
                obs(timestamp=now - 1.0),
                cfg,
                current_time=now,
            )
        )
        self.assertFalse(
            is_alignment_ready(
                obs(state=MissionState.HOLD),
                cfg,
                current_time=now,
            )
        )


if __name__ == '__main__':
    unittest.main()
