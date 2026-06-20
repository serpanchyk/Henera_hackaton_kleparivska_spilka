import importlib.util
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FOLLOWER_CONTROLLER_PATH = ROOT / 'drone_sdk' / 'follower_controller.py'
CV_SWARM_PATH = ROOT / 'resources' / 'scripts' / 'cv_swarm_test.py'


def _load_follower_controller():
    spec = importlib.util.spec_from_file_location(
        'follower_controller_for_cv_swarm_test',
        FOLLOWER_CONTROLLER_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_cv_swarm_module():
    follower_controller = _load_follower_controller()
    stubbed_modules = [
        'drone_sdk',
        'drone_sdk.follower_controller',
        'drone_sdk.two_led_cv',
        'rclpy',
        'henera_swarm',
        'henera_swarm.logging',
        'henera_swarm.perception',
    ]
    previous_modules = {
        name: sys.modules[name]
        for name in stubbed_modules
        if name in sys.modules
    }

    drone_sdk = types.ModuleType('drone_sdk')
    drone_sdk.Drone = object
    sys.modules['drone_sdk'] = drone_sdk
    sys.modules['drone_sdk.follower_controller'] = follower_controller

    two_led_cv = types.ModuleType('drone_sdk.two_led_cv')
    two_led_cv.mask_for_state = lambda state, _t: state
    sys.modules['drone_sdk.two_led_cv'] = two_led_cv

    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None
    sys.modules['rclpy'] = rclpy

    logging_module = types.ModuleType('henera_swarm.logging')
    logging_module.ResultsLogger = object
    perception_module = types.ModuleType('henera_swarm.perception')
    perception_module.CVVisionProvider = object
    henera_swarm = types.ModuleType('henera_swarm')
    sys.modules['henera_swarm'] = henera_swarm
    sys.modules['henera_swarm.logging'] = logging_module
    sys.modules['henera_swarm.perception'] = perception_module

    spec = importlib.util.spec_from_file_location('cv_swarm_test_under_test', CV_SWARM_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    try:
        spec.loader.exec_module(module)
    finally:
        for name in stubbed_modules:
            if name in previous_modules:
                sys.modules[name] = previous_modules[name]
            else:
                sys.modules.pop(name, None)
    return module, follower_controller


cv_swarm, follower_controller = _load_cv_swarm_module()
MissionState = follower_controller.MissionState
FollowerState = follower_controller.FollowerState
FollowerCommand = follower_controller.FollowerCommand
VisualObservation = follower_controller.VisualObservation


def obs(visible=True, state='UNKNOWN'):
    return VisualObservation(
        target_visible=visible,
        horizontal_angle_deg=12.0,
        vertical_angle_deg=-4.0,
        target_size=80.0,
        mission_state=state,
        timestamp=1.0,
    )


def command(state=FollowerState.FOLLOW, relay=MissionState.FOLLOW):
    return FollowerCommand(
        forward_m_s=0.0,
        right_m_s=0.0,
        down_m_s=0.0,
        yaw_rate_deg_s=0.0,
        relay_state=relay,
        state=state,
    )


class CVSwarmRuntimeHelperTests(unittest.TestCase):
    def test_visible_unknown_is_usable_for_navigation(self):
        normalized = cv_swarm._navigation_observation(obs(visible=True, state='UNKNOWN'))

        self.assertTrue(normalized.target_visible)
        self.assertEqual(normalized.mission_state, MissionState.FOLLOW)
        self.assertEqual(normalized.horizontal_angle_deg, 12.0)
        self.assertEqual(normalized.vertical_angle_deg, -4.0)

    def test_safe_and_finish_are_not_overridden(self):
        safe = cv_swarm._navigation_observation(obs(visible=True, state=MissionState.SAFE))
        finish = cv_swarm._navigation_observation(obs(visible=False, state=MissionState.FINISH))

        self.assertEqual(safe.mission_state, MissionState.SAFE)
        self.assertEqual(finish.mission_state, MissionState.FINISH)

    def test_beacon_stays_hold_until_raw_follow_is_confirmed(self):
        follow_command = command()

        self.assertEqual(cv_swarm._beacon_state(obs(True, 'UNKNOWN'), follow_command), 'HOLD')
        self.assertEqual(cv_swarm._beacon_state(obs(True, MissionState.HOLD), follow_command), 'HOLD')
        self.assertEqual(cv_swarm._beacon_state(obs(True, MissionState.FOLLOW), follow_command), 'FOLLOW')

    def test_chain_acquisition_requires_relay_ready(self):
        status = {
            1: {
                'state': FollowerState.FOLLOW,
                'visible': True,
                'size': 80.0,
                'relay_ready': False,
            }
        }

        self.assertFalse(cv_swarm._chain_acquired(status, [1]))
        status[1]['relay_ready'] = True
        self.assertTrue(cv_swarm._chain_acquired(status, [1]))


if __name__ == '__main__':
    unittest.main()
