import importlib.util
import sys
import types
import unittest
from pathlib import Path

PACKAGE_NAME = 'two_led_cv_decoder_under_test'
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


protocol = load_module('protocol')
decoder_module = load_module('decoder')

FINISH = protocol.FINISH
FOLLOW = protocol.FOLLOW
HOLD = protocol.HOLD
SAFE = protocol.SAFE
UNKNOWN = protocol.UNKNOWN
signal_on_for_state = protocol.signal_on_for_state

TwoLedCommandDecoder = decoder_module.TwoLedCommandDecoder
demo_generated_timestamps = decoder_module.demo_generated_timestamps


class TwoLedCommandDecoderTests(unittest.TestCase):

    def test_decodes_follow_safe_hold_and_finish_from_generated_timestamps(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t = 0.0

        t, decoded = self.feed_state(decoder, FOLLOW, t, duration_s=2.0)
        self.assertEqual(decoded, FOLLOW)

        t, decoded = self.feed_state(decoder, SAFE, t, duration_s=2.0)
        self.assertEqual(decoded, SAFE)

        t, decoded = self.feed_state(decoder, HOLD, t, duration_s=2.0)
        self.assertEqual(decoded, HOLD)

        _, decoded = self.feed_state(decoder, FINISH, t, duration_s=2.0)
        self.assertEqual(decoded, FINISH)

    def test_bad_anchor_visibility_keeps_previous_stable_state(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t, decoded = self.feed_state(decoder, FOLLOW, 0.0, duration_s=1.5)
        self.assertEqual(decoded, FOLLOW)

        for _ in range(30):
            decoded = decoder.update(anchor_visible=False, signal_visible=False, now=t)
            t += 0.05

        self.assertEqual(decoded, FOLLOW)
        self.assertEqual(decoder.debug_stats()['current_state'], FOLLOW)

    def test_noisy_unknown_window_does_not_replace_stable_state(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t, decoded = self.feed_state(decoder, SAFE, 0.0, duration_s=1.5)
        self.assertEqual(decoded, SAFE)

        noisy_pattern = [True, False, True, True, False, False, True]
        for index in range(30):
            decoded = decoder.update(
                anchor_visible=True,
                signal_visible=noisy_pattern[index % len(noisy_pattern)],
                now=t,
            )
            t += 0.05

        self.assertEqual(decoded, SAFE)

    def test_debug_stats_reports_current_window_values(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=4)
        _, decoded = self.feed_state(decoder, HOLD, 0.0, duration_s=1.5)

        stats = decoder.debug_stats()

        self.assertEqual(decoded, HOLD)
        self.assertGreaterEqual(stats['sample_count'], 4)
        self.assertEqual(stats['current_state'], HOLD)
        self.assertEqual(stats['candidate_state'], HOLD)
        self.assertAlmostEqual(stats['anchor_ratio'], 1.0)
        self.assertGreater(stats['transitions_per_s'], 1.0)

    def test_demo_generated_timestamps_runs_all_required_segments(self):
        rows = demo_generated_timestamps()
        commanded_states = {commanded for _, commanded, _ in rows}

        self.assertEqual(commanded_states, {FOLLOW, SAFE, HOLD, FINISH})
        self.assertGreater(len(rows), 100)

    def feed_state(self, decoder, state, start_t, duration_s, step_s=0.05):
        t = start_t
        decoded = UNKNOWN
        end_t = start_t + duration_s
        while t < end_t:
            decoded = decoder.update(
                anchor_visible=True,
                signal_visible=signal_on_for_state(state, t - start_t),
                now=t,
            )
            t += step_s
        return t, decoded


if __name__ == '__main__':
    unittest.main()
