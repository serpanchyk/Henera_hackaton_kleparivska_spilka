import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path

PACKAGE_NAME = 'two_led_cv_tracker_under_test'
PACKAGE_PATH = Path(__file__).resolve().parents[1] / 'drone_sdk' / 'two_led_cv'


def load_module(name):
    package = sys.modules.get(PACKAGE_NAME)
    if package is None:
        package = types.ModuleType(PACKAGE_NAME)
        package.__path__ = [str(PACKAGE_PATH)]
        sys.modules[PACKAGE_NAME] = package

    spec = importlib.util.spec_from_file_location(
        f'{PACKAGE_NAME}.{name}',
        PACKAGE_PATH / f'{name}.py',
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


types_module = load_module('types')
tracker_module = load_module('tracker')

LedBlob = types_module.LedBlob
TwoLedTracker = tracker_module.TwoLedTracker
demo_fake_blob_tracking = tracker_module.demo_fake_blob_tracking


class TwoLedTrackerTests(unittest.TestCase):

    def test_two_visible_blobs_return_midpoint_and_range(self):
        tracker = TwoLedTracker(frame_width=640, frame_height=480)

        observation = tracker.update(
            [
                LedBlob(cx=300.0, cy=220.0, area=80.0, confidence=0.8),
                LedBlob(cx=360.0, cy=260.0, area=90.0, confidence=1.0),
            ],
            state='FOLLOW',
            now=1.0,
        )

        self.assertTrue(observation.visible)
        self.assertTrue(observation.anchor_visible)
        self.assertTrue(observation.signal_visible)
        self.assertAlmostEqual(observation.x_error, 10.0 / 320.0)
        self.assertAlmostEqual(observation.y_error, 0.0)
        self.assertAlmostEqual(observation.led_distance_px, math.hypot(60.0, 40.0))
        self.assertAlmostEqual(observation.pair_angle_rad, math.atan2(40.0, 60.0))
        self.assertIsNotNone(observation.estimated_range_m)
        self.assertGreater(observation.estimated_range_m, 0.0)
        self.assertAlmostEqual(observation.confidence, 0.9)

    def test_one_visible_anchor_uses_recent_cached_vector(self):
        tracker = TwoLedTracker(frame_width=640, frame_height=480)
        pair = tracker.update(
            [
                LedBlob(cx=300.0, cy=220.0, area=80.0, confidence=0.8),
                LedBlob(cx=360.0, cy=260.0, area=90.0, confidence=1.0),
            ],
            state='FOLLOW',
            now=1.0,
        )

        fallback = tracker.update(
            [LedBlob(cx=302.0, cy=222.0, area=85.0, confidence=0.8)],
            state='SAFE',
            now=1.2,
        )

        self.assertTrue(fallback.visible)
        self.assertTrue(fallback.anchor_visible)
        self.assertFalse(fallback.signal_visible)
        self.assertAlmostEqual(fallback.x_error, 12.0 / 320.0)
        self.assertAlmostEqual(fallback.y_error, 2.0 / 240.0)
        self.assertEqual(fallback.led_distance_px, pair.led_distance_px)
        self.assertEqual(fallback.estimated_range_m, pair.estimated_range_m)
        self.assertLess(fallback.confidence, pair.confidence)

    def test_one_visible_anchor_without_recent_vector_is_not_visible(self):
        tracker = TwoLedTracker(frame_width=640, frame_height=480, last_vector_ttl_s=0.5)
        tracker.update(
            [
                LedBlob(cx=300.0, cy=220.0, area=80.0, confidence=0.8),
                LedBlob(cx=360.0, cy=260.0, area=90.0, confidence=1.0),
            ],
            state='FOLLOW',
            now=1.0,
        )

        observation = tracker.update(
            [LedBlob(cx=302.0, cy=222.0, area=85.0, confidence=0.8)],
            state='SAFE',
            now=2.0,
        )

        self.assertFalse(observation.visible)
        self.assertTrue(observation.anchor_visible)
        self.assertFalse(observation.signal_visible)
        self.assertIsNone(observation.estimated_range_m)

    def test_no_blobs_returns_invisible_with_age(self):
        tracker = TwoLedTracker(frame_width=640, frame_height=480)
        tracker.update(
            [
                LedBlob(cx=300.0, cy=220.0, area=80.0, confidence=0.8),
                LedBlob(cx=360.0, cy=260.0, area=90.0, confidence=1.0),
            ],
            state='FOLLOW',
            now=1.0,
        )

        observation = tracker.update([], state='SAFE', now=1.5)

        self.assertFalse(observation.visible)
        self.assertFalse(observation.anchor_visible)
        self.assertFalse(observation.signal_visible)
        self.assertEqual(observation.state, 'SAFE')
        self.assertIsNone(observation.x_error)
        self.assertIsNone(observation.y_error)
        self.assertIsNone(observation.led_distance_px)
        self.assertIsNone(observation.pair_angle_rad)
        self.assertIsNone(observation.estimated_range_m)
        self.assertAlmostEqual(observation.last_seen_age_s, 0.5)

    def test_invalid_pair_distance_is_rejected(self):
        tracker = TwoLedTracker(frame_width=640, frame_height=480, min_pair_distance_px=20.0)

        observation = tracker.update(
            [
                LedBlob(cx=300.0, cy=220.0, area=80.0, confidence=0.8),
                LedBlob(cx=305.0, cy=220.0, area=90.0, confidence=1.0),
            ],
            state='FOLLOW',
            now=1.0,
        )

        self.assertFalse(observation.visible)
        self.assertEqual(observation.confidence, 0.0)

    def test_range_decreases_when_pixel_distance_increases(self):
        tracker = TwoLedTracker(frame_width=640, frame_height=480)

        far = tracker.update(
            [
                LedBlob(cx=300.0, cy=240.0, area=80.0, confidence=0.8),
                LedBlob(cx=340.0, cy=240.0, area=90.0, confidence=0.8),
            ],
            state='FOLLOW',
            now=1.0,
        )
        near = tracker.update(
            [
                LedBlob(cx=280.0, cy=240.0, area=80.0, confidence=0.8),
                LedBlob(cx=360.0, cy=240.0, area=90.0, confidence=0.8),
            ],
            state='FOLLOW',
            now=2.0,
        )

        self.assertIsNotNone(far.estimated_range_m)
        self.assertIsNotNone(near.estimated_range_m)
        self.assertGreater(far.estimated_range_m, near.estimated_range_m)

    def test_demo_fake_blob_tracking_returns_sane_observations(self):
        pair, anchor_only, no_blobs = demo_fake_blob_tracking()

        self.assertTrue(pair.visible)
        self.assertTrue(anchor_only.visible)
        self.assertFalse(anchor_only.signal_visible)
        self.assertFalse(no_blobs.visible)
        self.assertEqual(no_blobs.state, 'SAFE')


if __name__ == '__main__':
    unittest.main()
