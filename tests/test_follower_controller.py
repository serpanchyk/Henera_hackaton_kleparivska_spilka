import importlib.util
import math
import sys
from pathlib import Path
import unittest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'drone_sdk' / 'follower_controller.py'
SPEC = importlib.util.spec_from_file_location('follower_controller_under_test', MODULE_PATH)
follower_controller = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = follower_controller
SPEC.loader.exec_module(follower_controller)

FollowerController = follower_controller.FollowerController
FollowerControllerConfig = follower_controller.FollowerControllerConfig
FollowerState = follower_controller.FollowerState
MissionState = follower_controller.MissionState
VisualObservation = follower_controller.VisualObservation
build_chain_config = follower_controller.build_chain_config
DroneFollowerActuator = follower_controller.DroneFollowerActuator
FollowerStartupConfig = follower_controller.FollowerStartupConfig
FollowerStartupError = follower_controller.FollowerStartupError
prepare_followers_for_chain = follower_controller.prepare_followers_for_chain
safe_stop_all = follower_controller.safe_stop_all


def obs(
    t=0.0,
    visible=True,
    h=0.0,
    v=0.0,
    size=70.0,
    state=MissionState.FOLLOW,
):
    return VisualObservation(
        target_visible=visible,
        horizontal_angle_deg=h,
        vertical_angle_deg=v,
        target_size=size,
        mission_state=state,
        timestamp=t,
    )


def config(**overrides):
    values = dict(
        desired_target_size=80.0,
        kp_yaw=2.0,
        kp_forward=0.1,
        kp_vertical=0.5,
        max_yaw_rate=20.0,
        max_forward_speed=3.0,
        max_vertical_speed=2.0,
        yaw_dead_zone_deg=1.0,
        vertical_dead_zone_deg=1.0,
        size_dead_zone=2.0,
        lost_timeout=0.5,
        observation_timeout=0.2,
        lost_frames_threshold=1,
        reacquire_frames=1,
        smoothing_alpha=0.0,
        search_yaw_rate=5.0,
        control_rate_hz=10.0,
        yaw_sign=1.0,
        vertical_sign=1.0,
    )
    values.update(overrides)
    return FollowerControllerConfig(**values)


def controller(**overrides):
    return FollowerController('follower_1', 'leader', config(**overrides))


class FollowerControllerTests(unittest.TestCase):

    def test_negative_horizontal_angle_yaws_negative(self):
        cmd = controller().update(obs(h=-5.0), current_time=0.0)
        self.assertLess(cmd.yaw_rate_deg_s, 0.0)
        self.assertEqual(cmd.relay_state, MissionState.FOLLOW)

    def test_positive_horizontal_angle_yaws_positive(self):
        cmd = controller().update(obs(h=5.0), current_time=0.0)
        self.assertGreater(cmd.yaw_rate_deg_s, 0.0)

    def test_negative_vertical_angle_commands_negative_down(self):
        cmd = controller().update(obs(v=-4.0), current_time=0.0)
        self.assertLess(cmd.down_m_s, 0.0)

    def test_positive_vertical_angle_commands_positive_down(self):
        cmd = controller().update(obs(v=4.0), current_time=0.0)
        self.assertGreater(cmd.down_m_s, 0.0)

    def test_small_target_moves_forward(self):
        cmd = controller().update(obs(size=60.0), current_time=0.0)
        self.assertGreater(cmd.forward_m_s, 0.0)
        self.assertEqual(cmd.size_error, 20.0)

    def test_large_target_moves_backward(self):
        cmd = controller().update(obs(size=100.0), current_time=0.0)
        self.assertLess(cmd.forward_m_s, 0.0)
        self.assertEqual(cmd.size_error, -20.0)

    def test_dead_zones_are_degrees_and_pixels(self):
        cmd = controller().update(obs(h=0.5, v=0.5, size=79.0), current_time=0.0)
        self.assertEqual(cmd.yaw_rate_deg_s, 0.0)
        self.assertEqual(cmd.down_m_s, 0.0)
        self.assertEqual(cmd.forward_m_s, 0.0)

    def test_clamp_limits_commands(self):
        cmd = controller().update(obs(h=100.0, v=100.0, size=0.0), current_time=0.0)
        self.assertEqual(cmd.yaw_rate_deg_s, 20.0)
        self.assertEqual(cmd.down_m_s, 2.0)
        self.assertEqual(cmd.forward_m_s, 3.0)

    def test_smoothing_blends_follow_commands(self):
        c = controller(smoothing_alpha=0.5)
        first = c.update(obs(h=10.0, size=60.0), current_time=0.0)
        second = c.update(obs(h=0.0, size=80.0), current_time=0.1)
        self.assertEqual(first.yaw_rate_deg_s, 20.0)
        self.assertEqual(first.forward_m_s, 2.0)
        self.assertEqual(second.yaw_rate_deg_s, 10.0)
        self.assertEqual(second.forward_m_s, 1.0)

    def test_nan_angle_blocks_motion_and_enters_lost_from_follow(self):
        c = controller()
        c.update(obs(), current_time=0.0)
        cmd = c.update(obs(t=0.1, h=math.nan), current_time=0.1)
        self.assertEqual(cmd.state, FollowerState.LOST)
        self.assertEqual(cmd.forward_m_s, 0.0)
        self.assertEqual(cmd.relay_state, MissionState.HOLD)

    def test_inf_angle_blocks_motion(self):
        c = controller()
        c.update(obs(), current_time=0.0)
        cmd = c.update(obs(t=0.1, v=math.inf), current_time=0.1)
        self.assertEqual(cmd.state, FollowerState.LOST)
        self.assertEqual(cmd.forward_m_s, 0.0)

    def test_invalid_size_blocks_motion(self):
        c = controller()
        c.update(obs(), current_time=0.0)
        cmd = c.update(obs(t=0.1, size=-1.0), current_time=0.1)
        self.assertEqual(cmd.state, FollowerState.LOST)
        self.assertEqual(cmd.forward_m_s, 0.0)

    def test_hold_has_priority_over_geometry(self):
        cmd = controller().update(
            obs(h=20.0, v=20.0, size=1.0, state=MissionState.HOLD),
            current_time=0.0,
        )
        self.assertEqual(cmd.state, FollowerState.HOLD)
        self.assertEqual(cmd.relay_state, MissionState.HOLD)
        self.assertEqual(cmd.forward_m_s, 0.0)
        self.assertEqual(cmd.down_m_s, 0.0)
        self.assertEqual(cmd.yaw_rate_deg_s, 0.0)


    def test_safe_has_priority_and_relays_safe(self):
        cmd = controller().update(
            obs(h=20.0, v=20.0, size=1.0, state=MissionState.SAFE),
            current_time=0.0,
        )
        self.assertEqual(cmd.state, FollowerState.HOLD)
        self.assertEqual(cmd.relay_state, MissionState.SAFE)
        self.assertEqual(cmd.forward_m_s, 0.0)
        self.assertEqual(cmd.down_m_s, 0.0)
        self.assertEqual(cmd.yaw_rate_deg_s, 0.0)

    def test_safe_is_not_terminal_and_recovers_after_follow_reacquire(self):
        c = controller(reacquire_frames=2)
        safe = c.update(obs(state=MissionState.SAFE), current_time=0.0)
        first_follow = c.update(obs(t=0.1), current_time=0.1)
        second_follow = c.update(obs(t=0.2), current_time=0.2)
        self.assertEqual(safe.relay_state, MissionState.SAFE)
        self.assertEqual(first_follow.state, FollowerState.HOLD)
        self.assertEqual(first_follow.relay_state, MissionState.HOLD)
        self.assertEqual(second_follow.state, FollowerState.FOLLOW)
        self.assertEqual(second_follow.relay_state, MissionState.FOLLOW)

    def test_finish_has_priority_and_is_terminal(self):
        c = controller()
        cmd = c.update(obs(state=MissionState.FINISH), current_time=0.0)
        self.assertEqual(cmd.state, FollowerState.FINISH)
        self.assertEqual(cmd.relay_state, MissionState.FINISH)
        again = c.update(obs(t=0.1, h=10.0, size=60.0), current_time=0.1)
        self.assertEqual(again.state, FollowerState.FINISH)
        self.assertEqual(again.relay_state, MissionState.FINISH)
        self.assertEqual(again.forward_m_s, 0.0)

    def test_stale_observation_enters_lost_from_follow(self):
        c = controller(observation_timeout=0.2)
        c.update(obs(t=0.0), current_time=0.0)
        cmd = c.update(obs(t=0.0), current_time=0.3)
        self.assertEqual(cmd.state, FollowerState.LOST)
        self.assertEqual(cmd.relay_state, MissionState.HOLD)

    def test_lost_transitions_to_search_after_lost_timeout(self):
        c = controller(lost_timeout=0.5)
        c.update(obs(t=0.0), current_time=0.0)
        c.update(obs(t=0.1, visible=False), current_time=0.1)
        cmd = c.update(obs(t=0.7, visible=False), current_time=0.7)
        self.assertEqual(cmd.state, FollowerState.SEARCH)
        self.assertEqual(cmd.relay_state, MissionState.HOLD)
        self.assertGreater(cmd.yaw_rate_deg_s, 0.0)

    def test_lost_can_briefly_preserve_last_yaw_and_vertical_correction(self):
        c = controller(smoothing_alpha=0.5, lost_command_memory_s=0.3)
        follow = c.update(obs(t=0.0, h=10.0, v=4.0), current_time=0.0)
        lost = c.update(obs(t=0.1, visible=False), current_time=0.1)
        expired = c.update(obs(t=0.5, visible=False), current_time=0.5)

        self.assertEqual(follow.state, FollowerState.FOLLOW)
        self.assertEqual(lost.state, FollowerState.LOST)
        self.assertGreater(lost.yaw_rate_deg_s, 0.0)
        self.assertGreater(lost.down_m_s, 0.0)
        self.assertEqual(expired.yaw_rate_deg_s, 0.0)
        self.assertEqual(expired.down_m_s, 0.0)

    def test_hold_restarts_search_when_follow_continues_without_visibility(self):
        c = controller()
        c.update(obs(state=MissionState.SAFE), current_time=0.0)
        cmd = c.update(obs(t=0.1, visible=False, state=MissionState.FOLLOW), current_time=0.1)
        self.assertEqual(cmd.state, FollowerState.SEARCH)
        self.assertEqual(cmd.relay_state, MissionState.HOLD)
        self.assertGreater(cmd.yaw_rate_deg_s, 0.0)

    def test_invalid_mission_state_goes_directly_to_hold(self):
        cmd = controller().update(obs(state='BROKEN'), current_time=0.0)
        self.assertEqual(cmd.state, FollowerState.HOLD)
        self.assertEqual(cmd.relay_state, MissionState.HOLD)

    def test_recovery_requires_reacquire_frames(self):
        c = controller(reacquire_frames=2)
        first = c.update(obs(t=0.0), current_time=0.0)
        second = c.update(obs(t=0.1), current_time=0.1)
        self.assertEqual(first.state, FollowerState.SEARCH)
        self.assertEqual(first.relay_state, MissionState.HOLD)
        self.assertEqual(second.state, FollowerState.FOLLOW)
        self.assertEqual(second.relay_state, MissionState.FOLLOW)

    def test_cascade_hold_blocks_next_follower(self):
        c1 = FollowerController('follower_1', 'leader', config())
        c2 = FollowerController('follower_2', 'follower_1', config())
        c1.update(obs(t=0.0), current_time=0.0)
        c2.update(obs(t=0.0, state=MissionState.FOLLOW), current_time=0.0)

        hold1 = c1.update(obs(t=0.1, visible=False), current_time=0.1)
        relay_to_follower_2 = obs(t=0.1, state=hold1.relay_state, size=60.0)
        hold2 = c2.update(relay_to_follower_2, current_time=0.1)

        self.assertEqual(hold1.state, FollowerState.LOST)
        self.assertEqual(hold1.relay_state, MissionState.HOLD)
        self.assertEqual(hold2.state, FollowerState.HOLD)
        self.assertEqual(hold2.forward_m_s, 0.0)

    def test_chain_mapping_for_2_3_5_followers(self):
        self.assertEqual(
            [(link.follower_id, link.target_id) for link in build_chain_config(2)],
            [('follower_1', 'leader'), ('follower_2', 'follower_1')],
        )
        self.assertEqual(build_chain_config(3)[2].target_id, 'follower_2')
        self.assertEqual(build_chain_config(5)[4].target_id, 'follower_4')
        # Longer chains are supported (each follows the previous drone).
        chain10 = build_chain_config(10)
        self.assertEqual(len(chain10), 10)
        self.assertEqual(chain10[9].target_id, 'follower_9')

    def test_chain_mapping_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            build_chain_config(1)
        with self.assertRaises(ValueError):
            build_chain_config(21)


class MockPosition:

    def __init__(self, down_m):
        self.down_m = down_m


class MockDrone:

    def __init__(self, height_m=10.0, fail_takeoff=False):
        self.height_m = height_m
        self.fail_takeoff = fail_takeoff
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


class StartupHelperTests(unittest.IsolatedAsyncioTestCase):

    async def test_all_followers_prepare_successfully(self):
        drones = [MockDrone(height_m=10.0), MockDrone(height_m=10.2)]
        startup = FollowerStartupConfig(
            startup_altitude_m=10.0,
            startup_timeout_s=0.05,
            startup_settle_time_s=0.0,
            startup_height_tolerance_m=0.5,
            startup_poll_interval_s=0.001,
        )
        await prepare_followers_for_chain(drones, startup)
        self.assertTrue(all(drone.armed for drone in drones))
        self.assertEqual([drone.takeoff_altitudes for drone in drones], [[10.0], [10.0]])
        self.assertTrue(all(drone.offboard_started for drone in drones))
        self.assertTrue(all(drone.velocity_commands for drone in drones))

    async def test_startup_failure_safe_stops_all_followers(self):
        drones = [MockDrone(height_m=10.0), MockDrone(height_m=10.0, fail_takeoff=True)]
        startup = FollowerStartupConfig(startup_settle_time_s=0.0)
        with self.assertRaises(FollowerStartupError):
            await prepare_followers_for_chain(drones, startup)
        self.assertTrue(all(drone.velocity_commands for drone in drones))

    async def test_startup_timeout_safe_stops_all_followers(self):
        drones = [MockDrone(height_m=0.0), MockDrone(height_m=10.0)]
        startup = FollowerStartupConfig(
            startup_altitude_m=10.0,
            startup_timeout_s=0.01,
            startup_settle_time_s=0.0,
            startup_height_tolerance_m=0.1,
            startup_poll_interval_s=0.001,
        )
        with self.assertRaises(FollowerStartupError):
            await prepare_followers_for_chain(drones, startup)
        self.assertTrue(all(drone.velocity_commands for drone in drones))

    async def test_safe_stop_all_sends_zero_velocity(self):
        drones = [MockDrone(height_m=10.0), MockDrone(height_m=10.0)]
        await safe_stop_all(drones)
        self.assertEqual(drones[0].velocity_commands[-1], (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(drones[1].velocity_commands[-1], (0.0, 0.0, 0.0, 0.0))


class ProfileAndActuatorTests(unittest.IsolatedAsyncioTestCase):

    def test_responsive_profile_has_higher_rate_and_less_smoothing(self):
        stable = FollowerControllerConfig.stable()
        responsive = FollowerControllerConfig.responsive()
        self.assertGreater(responsive.control_rate_hz, stable.control_rate_hz)
        self.assertLess(responsive.smoothing_alpha, stable.smoothing_alpha)
        self.assertEqual(responsive.reacquire_frames, 2)

    def test_responsive_profile_reacquires_faster_than_stable(self):
        stable_controller = FollowerController('follower_1', 'leader', FollowerControllerConfig.stable())
        responsive_controller = FollowerController('follower_1', 'leader', FollowerControllerConfig.responsive())
        stable_first = stable_controller.update(obs(t=0.0), current_time=0.0)
        stable_second = stable_controller.update(obs(t=0.1), current_time=0.1)
        responsive_first = responsive_controller.update(obs(t=0.0), current_time=0.0)
        responsive_second = responsive_controller.update(obs(t=0.1), current_time=0.1)
        self.assertNotEqual(stable_second.state, FollowerState.FOLLOW)
        self.assertEqual(responsive_first.state, FollowerState.SEARCH)
        self.assertEqual(responsive_second.state, FollowerState.FOLLOW)

    def test_search_scans_yaw_and_vertical_instead_of_spinning(self):
        c = controller(search_yaw_sweep_deg=30.0, search_vertical_speed=0.5, search_period_s=8.0)
        start = c.update(obs(visible=False, v=45.0), current_time=0.0)
        self.assertEqual(start.state, FollowerState.SEARCH)
        self.assertEqual(start.forward_m_s, 0.0)
        self.assertEqual(start.right_m_s, 0.0)
        # Scan starts centered: yaw begins turning immediately, no vertical jump.
        self.assertGreater(start.yaw_rate_deg_s, 0.0)
        self.assertEqual(start.down_m_s, 0.0)
        # A quarter period later yaw is at its sweep extreme (~0 rate) and the
        # up/down bob is at its peak.
        quarter = c.update(obs(visible=False), current_time=2.0)
        self.assertAlmostEqual(quarter.yaw_rate_deg_s, 0.0, places=4)
        self.assertGreater(quarter.down_m_s, 0.0)

    def test_search_yaw_sweep_reverses_and_stays_bounded(self):
        c = controller(search_yaw_sweep_deg=30.0, search_period_s=8.0, max_yaw_rate=40.0)
        rates = [
            c.update(obs(visible=False), current_time=i * 0.1).yaw_rate_deg_s
            for i in range(81)  # one full 8 s sweep cycle
        ]
        self.assertGreater(max(rates), 0.0)   # turns one way...
        self.assertLess(min(rates), 0.0)      # ...then reverses (no full 360 spin)
        self.assertTrue(all(abs(rate) <= 40.0 for rate in rates))

    def test_different_start_height_is_not_randomly_compensated_in_search(self):
        c = controller()
        cmd = c.update(obs(visible=False, v=-45.0, size=0.0), current_time=0.0)
        self.assertEqual(cmd.down_m_s, 0.0)
        self.assertEqual(cmd.forward_m_s, 0.0)

    async def test_yaw_integration_dt_is_capped(self):
        drone = MockDrone(height_m=10.0)
        actuator = DroneFollowerActuator(drone, max_yaw_integration_dt=0.1)
        command = follower_controller.FollowerCommand(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=0.0,
            yaw_rate_deg_s=30.0,
            relay_state=MissionState.FOLLOW,
            state=FollowerState.FOLLOW,
        )
        await actuator.apply(command, current_time=0.0)
        await actuator.apply(command, current_time=1.0)
        self.assertEqual(drone.velocity_commands[-1][3], 3.0)

    def test_responsive_profile_still_clamps_commands(self):
        c = FollowerController('follower_1', 'leader', FollowerControllerConfig.responsive())
        cmd = c.update(obs(h=1000.0, v=1000.0, size=0.0), current_time=0.0)
        self.assertLessEqual(abs(cmd.yaw_rate_deg_s), c.config.max_yaw_rate)
        self.assertLessEqual(abs(cmd.down_m_s), c.config.max_vertical_speed)
        self.assertLessEqual(abs(cmd.forward_m_s), c.config.max_forward_speed)

    def test_too_close_backs_off_only_at_reverse_speed(self):
        # Target much bigger than desired -> too close -> must back off, but only
        # at the (smaller) reverse cap, never at full forward speed. This is the
        # fix for the dangerous full-speed fly-back into the trailing drone.
        c = FollowerController(
            'follower_1', 'leader',
            config(max_forward_speed=3.0, max_reverse_speed=0.5),
        )
        cmd = c.update(obs(h=0.0, v=0.0, size=10_000.0), current_time=0.0)
        self.assertLess(cmd.forward_m_s, 0.0)  # backing off
        self.assertGreaterEqual(cmd.forward_m_s, -0.5)  # clamped to reverse cap
        self.assertLess(abs(cmd.forward_m_s), 3.0)  # never full forward magnitude

    def test_approach_still_uses_full_forward_speed(self):
        # Far target (small size) -> approach is clamped to max_forward_speed,
        # unaffected by the smaller reverse cap.
        c = FollowerController(
            'follower_1', 'leader',
            config(max_forward_speed=3.0, max_reverse_speed=0.5),
        )
        cmd = c.update(obs(h=0.0, v=0.0, size=0.0), current_time=0.0)
        self.assertEqual(cmd.forward_m_s, 3.0)

    def test_matched_config_ties_follower_speeds_to_leader_cruise(self):
        cfg = FollowerControllerConfig.matched(2.0)
        # Forward > leader cruise (can close a gap) but not a runaway multiple;
        # reverse strictly below forward (gentle back-off).
        self.assertGreater(cfg.max_forward_speed, 2.0)
        self.assertLess(cfg.max_reverse_speed, cfg.max_forward_speed)
        # Overrides still apply on top of the matched speeds.
        self.assertEqual(FollowerControllerConfig.matched(2.0, kp_forward=0.09).kp_forward, 0.09)

    def test_hold_and_finish_remain_immediate_zero_commands(self):
        hold = FollowerController('follower_1', 'leader', FollowerControllerConfig.responsive()).update(
            obs(h=1000.0, v=1000.0, size=0.0, state=MissionState.HOLD),
            current_time=0.0,
        )
        finish = FollowerController('follower_1', 'leader', FollowerControllerConfig.responsive()).update(
            obs(h=1000.0, v=1000.0, size=0.0, state=MissionState.FINISH),
            current_time=0.0,
        )
        self.assertEqual((hold.forward_m_s, hold.down_m_s, hold.yaw_rate_deg_s), (0.0, 0.0, 0.0))
        self.assertEqual((finish.forward_m_s, finish.down_m_s, finish.yaw_rate_deg_s), (0.0, 0.0, 0.0))



if __name__ == '__main__':
    unittest.main()
