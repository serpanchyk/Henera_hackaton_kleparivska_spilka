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
led_states_for_state = protocol.led_states_for_state

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

    def test_noisy_finish_blink_decodes_without_perfect_regular_intervals(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t, decoded = self.feed_state(decoder, FOLLOW, 0.0, duration_s=1.5)
        self.assertEqual(decoded, FOLLOW)

        transition_times = [0.52, 1.03, 1.57, 2.06, 2.54, 3.08, 3.55, 4.04]
        transition_index = 0
        both_visible = True
        for index in range(45):
            offset = index * 0.1
            while transition_index < len(transition_times) and offset >= transition_times[transition_index]:
                both_visible = not both_visible
                transition_index += 1
            decoded = decoder.update(
                anchor_visible=both_visible,
                signal_visible=both_visible,
                now=t + offset,
            )

        self.assertEqual(decoded, FINISH)

    def test_finish_latches_after_later_noisy_or_safe_samples(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t, decoded = self.feed_state(decoder, FINISH, 0.0, duration_s=2.0)
        self.assertEqual(decoded, FINISH)

        for _ in range(30):
            decoded = decoder.update(anchor_visible=True, signal_visible=False, now=t)
            t += 0.05

        self.assertEqual(decoded, FINISH)
        self.assertEqual(decoder.debug_stats()['current_state'], FINISH)

    def test_debug_stats_reports_current_window_values(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=4)
        _, decoded = self.feed_state(decoder, HOLD, 0.0, duration_s=1.5)

        stats = decoder.debug_stats()

        self.assertEqual(decoded, HOLD)
        self.assertGreaterEqual(stats['sample_count'], 4)
        self.assertEqual(stats['current_state'], HOLD)
        self.assertEqual(stats['candidate_state'], HOLD)
        self.assertAlmostEqual(stats['anchor_ratio'], 1.0)
        self.assertAlmostEqual(stats['red_on_ratio'], 0.0)
        self.assertEqual(stats['green_transition_count'], 0)
        self.assertEqual(stats['red_transition_count'], 0)

    def test_relayed_follow_with_single_frame_dropouts_stays_follow(self):
        # Regression: a relayed FOLLOW link drops the signal LED for one frame here
        # and there (detection noise at ~4 m). With the decoder fed already-debounced
        # input this is just steady FOLLOW; raw 1-frame red dropouts must never be read as
        # synchronized green+red FINISH blinking.
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t, decoded = self.feed_state(decoder, FOLLOW, 0.0, duration_s=1.5)
        self.assertEqual(decoded, FOLLOW)

        # Solid ON with an isolated single-frame dropout every ~0.5 s.
        for index in range(60):
            signal_visible = not (index % 10 == 0)  # one frame off in every ten
            decoded = decoder.update(anchor_visible=True, signal_visible=signal_visible, now=t)
            t += 0.05

        self.assertEqual(decoded, FOLLOW)
        self.assertNotEqual(decoder.debug_stats()['current_state'], FINISH)

    def test_finish_requires_sustained_confirmation_before_committing(self):
        # A real leader FINISH (both LEDs toggle together) still commits, but only after the
        # raised confirmation threshold — a brief noisy burst must not flip to FINISH.
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t, decoded = self.feed_state(decoder, FOLLOW, 0.0, duration_s=1.5)
        self.assertEqual(decoded, FOLLOW)

        _, decoded = self.feed_state(decoder, FINISH, t, duration_s=3.0)
        self.assertEqual(decoded, FINISH)

    def test_out_of_phase_blink_is_not_finish(self):
        decoder = TwoLedCommandDecoder(window_s=1.0, min_samples=8)
        t, decoded = self.feed_state(decoder, FOLLOW, 0.0, duration_s=1.5)
        self.assertEqual(decoded, FOLLOW)

        for index in range(60):
            green_on = (index // 10) % 2 == 0
            red_on = not green_on
            decoded = decoder.update(anchor_visible=green_on, signal_visible=red_on, now=t)
            t += 0.05

        self.assertNotEqual(decoded, FINISH)
        self.assertNotEqual(decoder.debug_stats()['current_state'], FINISH)

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
            green_on, red_on = led_states_for_state(state, t - start_t)
            decoded = decoder.update(
                anchor_visible=green_on,
                signal_visible=red_on,
                now=t,
            )
            t += step_s
        return t, decoded


if __name__ == '__main__':
    unittest.main()
