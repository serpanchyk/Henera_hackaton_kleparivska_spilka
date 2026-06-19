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

    def test_lost_transitions_to_hold_after_lost_timeout(self):
        c = controller(lost_timeout=0.5)
        c.update(obs(t=0.0), current_time=0.0)
        c.update(obs(t=0.1, visible=False), current_time=0.1)
        cmd = c.update(obs(t=0.7, visible=False), current_time=0.7)
        self.assertEqual(cmd.state, FollowerState.HOLD)
        self.assertEqual(cmd.relay_state, MissionState.HOLD)

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

    def test_chain_mapping_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            build_chain_config(1)
        with self.assertRaises(ValueError):
            build_chain_config(6)


if __name__ == '__main__':
    unittest.main()
