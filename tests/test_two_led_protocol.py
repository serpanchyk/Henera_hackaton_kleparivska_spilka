import unittest
import importlib.util
import sys
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / 'drone_sdk' / 'two_led_cv' / 'protocol.py'
SPEC = importlib.util.spec_from_file_location('two_led_protocol_under_test', MODULE_PATH)
protocol = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = protocol
SPEC.loader.exec_module(protocol)

FINISH = protocol.FINISH
FOLLOW = protocol.FOLLOW
HOLD = protocol.HOLD
SAFE = protocol.SAFE
make_led_mask = protocol.make_led_mask
signal_on_for_state = protocol.signal_on_for_state


class TwoLedProtocolTests(unittest.TestCase):

    def test_make_led_mask_anchor_and_signal_on(self):
        self.assertEqual(make_led_mask(True, True), '1100')

    def test_make_led_mask_anchor_on_signal_off(self):
        self.assertEqual(make_led_mask(True, False), '1000')

    def test_follow_signal_is_always_on(self):
        for t in [0.0, 0.1, 0.5, 1.2, 10.0]:
            self.assertTrue(signal_on_for_state(FOLLOW, t))

    def test_safe_signal_is_always_off(self):
        for t in [0.0, 0.1, 0.5, 1.2, 10.0]:
            self.assertFalse(signal_on_for_state(SAFE, t))

    def test_unknown_state_behaves_as_safe(self):
        self.assertFalse(signal_on_for_state('BROKEN', 0.0))
        self.assertFalse(signal_on_for_state('BROKEN', 1.0))

    def test_hold_toggles_every_half_second(self):
        self.assertTrue(signal_on_for_state(HOLD, 0.0))
        self.assertTrue(signal_on_for_state(HOLD, 0.49))
        self.assertFalse(signal_on_for_state(HOLD, 0.5))
        self.assertFalse(signal_on_for_state(HOLD, 0.99))
        self.assertTrue(signal_on_for_state(HOLD, 1.0))

    def test_finish_toggles_every_point_two_seconds(self):
        self.assertTrue(signal_on_for_state(FINISH, 0.0))
        self.assertTrue(signal_on_for_state(FINISH, 0.19))
        self.assertFalse(signal_on_for_state(FINISH, 0.2))
        self.assertFalse(signal_on_for_state(FINISH, 0.39))
        self.assertTrue(signal_on_for_state(FINISH, 0.4))


if __name__ == '__main__':
    unittest.main()
