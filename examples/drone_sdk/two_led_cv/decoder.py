from __future__ import annotations

import time
from dataclasses import dataclass

from .protocol import FINISH, FOLLOW, HOLD, SAFE, UNKNOWN, led_states_for_state


@dataclass
class LedSample:
    t: float
    anchor_visible: bool
    signal_visible: bool


class TwoLedCommandDecoder:
    """Decode leader mission state from fixed green/red LED visibility."""

    # Most states commit after 2 confirming windows. FINISH is terminal (it latches
    # and stops the follower), so it must clear a much higher bar to survive the
    # transient blink noise on a relayed link that previously produced false FINISH.
    DEFAULT_COMMIT_COUNT = 2
    FINISH_COMMIT_COUNT = 5

    def __init__(
        self,
        window_s: float = 2.0,
        min_samples: int = 10,
        finish_latches: bool = True,
        finish_commit_count: int = FINISH_COMMIT_COUNT,
    ) -> None:
        if window_s <= 0.0:
            raise ValueError('window_s must be positive')
        if min_samples <= 0:
            raise ValueError('min_samples must be positive')

        self.window_s = float(window_s)
        self.min_samples = int(min_samples)
        self.samples: list[LedSample] = []
        self.current_state = UNKNOWN
        self.candidate_state = UNKNOWN
        self.candidate_count = 0
        self.finish_latches = bool(finish_latches)
        self.finish_commit_count = max(1, int(finish_commit_count))

    def _commit_threshold(self, decoded: str) -> int:
        return self.finish_commit_count if decoded == FINISH else self.DEFAULT_COMMIT_COUNT

    def update(
        self,
        anchor_visible: bool,
        signal_visible: bool,
        now: float | None = None,
    ) -> str:
        timestamp = time.monotonic() if now is None else float(now)
        self.samples.append(
            LedSample(
                t=timestamp,
                anchor_visible=bool(anchor_visible),
                signal_visible=bool(signal_visible),
            )
        )
        self._prune(timestamp)

        if self.finish_latches and self.current_state == FINISH:
            return self.current_state

        decoded = self._decode_window()
        if decoded == UNKNOWN:
            return self.current_state

        if decoded == self.current_state:
            self.candidate_state = decoded
            self.candidate_count = 0
            return self.current_state

        if decoded == self.candidate_state:
            self.candidate_count += 1
        else:
            self.candidate_state = decoded
            self.candidate_count = 1

        if self.candidate_count >= self._commit_threshold(decoded):
            self.current_state = decoded
            self.candidate_count = 0

        return self.current_state

    def debug_stats(self) -> dict[str, float | int | str]:
        green_ratio, red_ratio, transitions_per_s = self._window_stats()
        return {
            'sample_count': len(self.samples),
            'anchor_ratio': green_ratio,
            'signal_on_ratio': red_ratio,
            'green_on_ratio': green_ratio,
            'red_on_ratio': red_ratio,
            'transition_count': self._count_red_transitions(),
            'green_transition_count': self._count_green_transitions(),
            'red_transition_count': self._count_red_transitions(),
            'co_transition_count': self._count_co_transitions(),
            'transitions_per_s': transitions_per_s,
            'current_state': self.current_state,
            'candidate_state': self.candidate_state,
            'candidate_count': self.candidate_count,
        }

    def _prune(self, timestamp: float) -> None:
        cutoff = timestamp - self.window_s
        self.samples = [sample for sample in self.samples if sample.t >= cutoff]

    def _decode_window(self) -> str:
        if len(self.samples) < self.min_samples:
            return UNKNOWN

        green_ratio, red_ratio, transitions_per_s = self._window_stats()
        green_transitions = self._count_green_transitions()
        red_transitions = self._count_red_transitions()
        co_transitions = self._count_co_transitions()
        steady = green_transitions == 0 and red_transitions == 0

        if green_ratio > 0.85 and red_ratio > 0.85 and steady:
            return FOLLOW
        if green_ratio > 0.85 and red_ratio < 0.15 and steady:
            return HOLD
        if green_ratio < 0.15 and red_ratio > 0.85 and steady:
            return SAFE
        if self._has_synchronized_blink(
            green_ratio=green_ratio,
            red_ratio=red_ratio,
            green_transitions=green_transitions,
            red_transitions=red_transitions,
            co_transitions=co_transitions,
            transitions_per_s=transitions_per_s,
        ):
            return FINISH
        return UNKNOWN

    def _window_stats(self) -> tuple[float, float, float]:
        if not self.samples:
            return 0.0, 0.0, 0.0

        sample_count = len(self.samples)
        green_ratio = sum(1 for sample in self.samples if sample.anchor_visible) / sample_count
        red_ratio = sum(1 for sample in self.samples if sample.signal_visible) / sample_count
        transitions = self._count_co_transitions()
        duration = max(0.0, self.samples[-1].t - self.samples[0].t)
        transitions_per_s = transitions / duration if duration > 0.0 else 0.0
        return green_ratio, red_ratio, transitions_per_s

    def _count_green_transitions(self) -> int:
        if not self.samples:
            return 0
        transitions = 0
        previous = self.samples[0].anchor_visible
        for sample in self.samples[1:]:
            if sample.anchor_visible != previous:
                transitions += 1
                previous = sample.anchor_visible
        return transitions

    def _count_red_transitions(self) -> int:
        if not self.samples:
            return 0
        transitions = 0
        previous = self.samples[0].signal_visible
        for sample in self.samples[1:]:
            if sample.signal_visible != previous:
                transitions += 1
                previous = sample.signal_visible
        return transitions

    def _count_co_transitions(self) -> int:
        if not self.samples:
            return 0
        transitions = 0
        previous_green = self.samples[0].anchor_visible
        previous_red = self.samples[0].signal_visible
        for sample in self.samples[1:]:
            green_changed = sample.anchor_visible != previous_green
            red_changed = sample.signal_visible != previous_red
            if green_changed and red_changed:
                transitions += 1
            previous_green = sample.anchor_visible
            previous_red = sample.signal_visible
        return transitions

    def _has_synchronized_blink(
        self,
        *,
        green_ratio: float,
        red_ratio: float,
        green_transitions: int,
        red_transitions: int,
        co_transitions: int,
        transitions_per_s: float,
    ) -> bool:
        balanced = 0.20 <= green_ratio <= 0.80 and 0.20 <= red_ratio <= 0.80
        if not balanced:
            return False
        if green_transitions < 2 or red_transitions < 2 or co_transitions < 2:
            return False
        if green_transitions != red_transitions or co_transitions != green_transitions:
            return False
        if not 0.8 <= transitions_per_s <= 3.2:
            return False
        same_phase_ratio = sum(
            1 for sample in self.samples
            if sample.anchor_visible == sample.signal_visible
        ) / len(self.samples)
        return same_phase_ratio >= 0.85



def demo_generated_timestamps() -> list[tuple[float, str, str]]:
    """Simulate the four LED4 command patterns without sleeping."""

    decoder = TwoLedCommandDecoder()
    rows: list[tuple[float, str, str]] = []
    t = 0.0
    step_s = 0.05
    for state in [FOLLOW, SAFE, HOLD, FINISH]:
        segment_end = t + 2.0
        while t < segment_end:
            green_on, red_on = led_states_for_state(state, t)
            decoded = decoder.update(anchor_visible=green_on, signal_visible=red_on, now=t)
            rows.append((round(t, 2), state, decoded))
            t += step_s
    return rows


if __name__ == '__main__':
    for timestamp, commanded, decoded in demo_generated_timestamps():
        print(f'{timestamp:5.2f}s commanded={commanded:7s} decoded={decoded}')
