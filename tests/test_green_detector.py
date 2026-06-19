import importlib.util
import sys
import types
import unittest
from pathlib import Path

import cv2
import numpy as np

PACKAGE_NAME = 'two_led_cv_detector_under_test'
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


load_module('types')
green_detector = load_module('green_detector')

GreenLedDetector = green_detector.GreenLedDetector
draw_debug = green_detector.draw_debug


class GreenLedDetectorTests(unittest.TestCase):

    def test_detects_green_led_blobs_sorted_by_confidence(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        cv2.circle(frame, (50, 60), 7, (0, 255, 0), -1)
        cv2.circle(frame, (100, 60), 5, (0, 220, 0), -1)

        blobs = GreenLedDetector().detect(frame)

        self.assertEqual(len(blobs), 2)
        self.assertAlmostEqual(blobs[0].cx, 50.0, delta=0.5)
        self.assertAlmostEqual(blobs[0].cy, 60.0, delta=0.5)
        self.assertGreaterEqual(blobs[0].confidence, blobs[1].confidence)

    def test_rejects_elongated_green_shapes(self):
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        cv2.rectangle(frame, (20, 55), (120, 60), (0, 255, 0), -1)

        blobs = GreenLedDetector().detect(frame)

        self.assertEqual(blobs, [])

    def test_draw_debug_returns_annotated_copy(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        cv2.circle(frame, (40, 40), 6, (0, 255, 0), -1)
        blobs = GreenLedDetector().detect(frame)

        debug = draw_debug(frame, blobs)

        self.assertEqual(debug.shape, frame.shape)
        self.assertGreater(int(debug.sum()), int(frame.sum()))


if __name__ == '__main__':
    unittest.main()
