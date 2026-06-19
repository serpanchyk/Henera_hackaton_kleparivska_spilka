import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
PKG = 'dsk_under_test'


def _ensure_pkg(name, path):
    pkg = sys.modules.get(name)
    if pkg is None:
        pkg = types.ModuleType(name)
        pkg.__path__ = [str(path)]
        sys.modules[name] = pkg
    return pkg


def _load(modname, filepath):
    spec = importlib.util.spec_from_file_location(modname, filepath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[modname] = module
    spec.loader.exec_module(module)
    return module


# Stand-in 'drone_sdk' package so adapter's `from ..follower_controller` resolves,
# without importing the real drone_sdk/__init__ (which pulls in rclpy).
_ensure_pkg(PKG, ROOT / 'drone_sdk')
_ensure_pkg(f'{PKG}.two_led_cv', ROOT / 'drone_sdk' / 'two_led_cv')
fc = _load(f'{PKG}.follower_controller', ROOT / 'drone_sdk' / 'follower_controller.py')
adapter = _load(f'{PKG}.two_led_cv.adapter', ROOT / 'drone_sdk' / 'two_led_cv' / 'adapter.py')

two_led_to_visual = adapter.two_led_to_visual
VisualObservation = fc.VisualObservation


def obs(visible=True, x=0.0, y=0.0, rng=3.0, state='FOLLOW'):
    return SimpleNamespace(
        visible=visible,
        x_error=x,
        y_error=y,
        estimated_range_m=rng,
        led_distance_px=20.0,
        state=state,
    )


class AdapterTests(unittest.TestCase):
    def test_returns_visual_observation(self):
        v = two_led_to_visual(obs())
        self.assertIsInstance(v, VisualObservation)

    def test_centered_target_is_zero_angle(self):
        v = two_led_to_visual(obs(x=0.0, y=0.0))
        self.assertAlmostEqual(v.horizontal_angle_deg, 0.0, places=6)
        self.assertAlmostEqual(v.vertical_angle_deg, 0.0, places=6)

    def test_right_target_yields_positive_yaw_angle(self):
        v = two_led_to_visual(obs(x=0.5))
        self.assertGreater(v.horizontal_angle_deg, 0.0)
        # exact pinhole value: atan(0.5 * tan(0.8))
        expected = math.degrees(math.atan(0.5 * math.tan(1.6 / 2)))
        self.assertAlmostEqual(v.horizontal_angle_deg, expected, places=4)

    def test_left_target_yields_negative_yaw_angle(self):
        v = two_led_to_visual(obs(x=-0.5))
        self.assertLess(v.horizontal_angle_deg, 0.0)

    def test_below_target_yields_positive_down_angle(self):
        v = two_led_to_visual(obs(y=0.5))
        self.assertGreater(v.vertical_angle_deg, 0.0)

    def test_full_fov_edge_maps_to_half_fov(self):
        # x_error = 1.0 -> bearing == hfov/2
        v = two_led_to_visual(obs(x=1.0))
        self.assertAlmostEqual(v.horizontal_angle_deg, math.degrees(1.6 / 2), places=4)

    def test_desired_distance_maps_to_desired_size(self):
        v = two_led_to_visual(obs(rng=3.0), size_gain=240.0)
        self.assertAlmostEqual(v.target_size, 80.0, places=3)

    def test_closer_target_is_bigger_size(self):
        near = two_led_to_visual(obs(rng=1.5))
        far = two_led_to_visual(obs(rng=6.0))
        self.assertGreater(near.target_size, far.target_size)

    def test_invisible_returns_safe_placeholders(self):
        v = two_led_to_visual(obs(visible=False, x=None, y=None, rng=None))
        self.assertFalse(v.target_visible)
        self.assertEqual(v.horizontal_angle_deg, 0.0)
        self.assertEqual(v.vertical_angle_deg, 0.0)
        self.assertEqual(v.target_size, 0.0)

    def test_state_passed_through(self):
        for s in ('FOLLOW', 'HOLD', 'FINISH', 'SAFE', 'UNKNOWN'):
            self.assertEqual(two_led_to_visual(obs(state=s)).mission_state, s)

    def test_missing_range_falls_back_to_desired_size(self):
        v = two_led_to_visual(obs(rng=None), desired_target_size=80.0)
        self.assertTrue(v.target_visible)
        self.assertAlmostEqual(v.target_size, 80.0, places=6)

    def test_timestamp_is_set(self):
        self.assertGreater(two_led_to_visual(obs()).timestamp, 0.0)


if __name__ == '__main__':
    unittest.main()
