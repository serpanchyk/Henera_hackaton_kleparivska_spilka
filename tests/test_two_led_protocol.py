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
led_states_for_state = protocol.led_states_for_state
make_led_mask = protocol.make_led_mask
mask_for_state = protocol.mask_for_state
signal_on_for_state = protocol.signal_on_for_state


class TwoLedProtocolTests(unittest.TestCase):

    def test_make_led_mask_anchor_and_signal_on(self):
        self.assertEqual(make_led_mask(True, True), '1100')

    def test_make_led_mask_anchor_on_signal_off(self):
        self.assertEqual(make_led_mask(True, False), '1000')

    def test_protocol_masks(self):
        self.assertEqual(protocol.FOLLOW_MASK, '1100')
        self.assertEqual(protocol.HOLD_MASK, '1000')
        self.assertEqual(protocol.SAFE_MASK, '0100')
        self.assertEqual(protocol.FINISH_ON_MASK, '1100')
        self.assertEqual(protocol.FINISH_OFF_MASK, '0000')

    def test_follow_signal_is_always_on(self):
        for t in [0.0, 0.1, 0.5, 1.2, 10.0]:
            self.assertEqual(led_states_for_state(FOLLOW, t), (True, True))
            self.assertEqual(mask_for_state(FOLLOW, t), '1100')

    def test_hold_is_green_only(self):
        for t in [0.0, 0.1, 0.5, 1.2, 10.0]:
            self.assertEqual(led_states_for_state(HOLD, t), (True, False))
            self.assertFalse(signal_on_for_state(HOLD, t))
            self.assertEqual(mask_for_state(HOLD, t), '1000')

    def test_safe_is_red_only(self):
        for t in [0.0, 0.1, 0.5, 1.2, 10.0]:
            self.assertEqual(led_states_for_state(SAFE, t), (False, True))
            self.assertTrue(signal_on_for_state(SAFE, t))
            self.assertEqual(mask_for_state(SAFE, t), '0100')

    def test_unknown_state_behaves_as_safe(self):
        self.assertEqual(led_states_for_state('BROKEN', 0.0), (False, False))
        self.assertEqual(mask_for_state('BROKEN', 1.0), '0000')

    def test_finish_toggles_both_leds_together_at_one_hz(self):
        self.assertEqual(led_states_for_state(FINISH, 0.0), (True, True))
        self.assertEqual(mask_for_state(FINISH, 0.49), '1100')
        self.assertEqual(led_states_for_state(FINISH, 0.5), (False, False))
        self.assertEqual(mask_for_state(FINISH, 0.99), '0000')
        self.assertEqual(led_states_for_state(FINISH, 1.0), (True, True))


if __name__ == '__main__':
    unittest.main()
