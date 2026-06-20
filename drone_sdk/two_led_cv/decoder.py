from __future__ import annotations

import time
from dataclasses import dataclass

from .protocol import FINISH, FOLLOW, HOLD, SAFE, UNKNOWN, signal_on_for_state


@dataclass
class LedSample:
    t: float
    anchor_visible: bool
    signal_visible: bool


class TwoLedCommandDecoder:
    """Decode leader mission state from LED 4 visibility over time."""

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
        anchor_ratio, signal_on_ratio, transitions_per_s = self._window_stats()
        return {
            'sample_count': len(self.samples),
            'anchor_ratio': anchor_ratio,
            'signal_on_ratio': signal_on_ratio,
            'transition_count': self._count_transitions(),
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

        anchor_ratio, signal_on_ratio, transitions_per_s = self._window_stats()
        transition_count = self._count_transitions()
        balanced_blink = 0.20 <= signal_on_ratio <= 0.80
        regular_blink = self._has_regular_transitions()
        if anchor_ratio < 0.6:
            return UNKNOWN

        if signal_on_ratio > 0.85 and transitions_per_s < 1.0:
            return FOLLOW
        if signal_on_ratio < 0.15 and transitions_per_s < 1.0:
            return SAFE
        if (
            transition_count >= 3
            and balanced_blink
            and 3.3 <= transitions_per_s <= 7.5
            and self._has_finish_transitions()
        ):
            return FINISH
        if transition_count >= 2 and balanced_blink and regular_blink and 1.0 <= transitions_per_s <= 3.2:
            return HOLD
        return UNKNOWN

    def _window_stats(self) -> tuple[float, float, float]:
        if not self.samples:
            return 0.0, 0.0, 0.0

        sample_count = len(self.samples)
        anchor_ratio = sum(1 for sample in self.samples if sample.anchor_visible) / sample_count
        signal_on_ratio = sum(1 for sample in self.samples if sample.signal_visible) / sample_count
        transitions = self._count_transitions()
        duration = max(0.0, self.samples[-1].t - self.samples[0].t)
        transitions_per_s = transitions / duration if duration > 0.0 else 0.0
        return anchor_ratio, signal_on_ratio, transitions_per_s

    def _count_transitions(self) -> int:
        if not self.samples:
            return 0
        transitions = 0
        previous = self.samples[0].signal_visible
        for sample in self.samples[1:]:
            if sample.signal_visible != previous:
                transitions += 1
                previous = sample.signal_visible
        return transitions

    def _has_regular_transitions(self) -> bool:
        transition_times: list[float] = []
        previous = self.samples[0].signal_visible
        for sample in self.samples[1:]:
            if sample.signal_visible != previous:
                transition_times.append(sample.t)
                previous = sample.signal_visible

        if len(transition_times) < 2:
            return False

        intervals = [
            transition_times[index] - transition_times[index - 1]
            for index in range(1, len(transition_times))
        ]
        average_interval = sum(intervals) / len(intervals)
        if average_interval <= 0.0:
            return False

        max_deviation = max(abs(interval - average_interval) for interval in intervals)
        return max_deviation <= max(0.03, average_interval * 0.25)

    def _has_finish_transitions(self) -> bool:
        transition_times: list[float] = []
        previous = self.samples[0].signal_visible
        for sample in self.samples[1:]:
            if sample.signal_visible != previous:
                transition_times.append(sample.t)
                previous = sample.signal_visible

        if len(transition_times) < 4:
            return False

        intervals = [
            transition_times[index] - transition_times[index - 1]
            for index in range(1, len(transition_times))
        ]
        average_interval = sum(intervals) / len(intervals)
        if not 0.12 <= average_interval <= 0.35:
            return False

        max_deviation = max(abs(interval - average_interval) for interval in intervals)
        return max_deviation <= max(0.08, average_interval * 0.45)


def demo_generated_timestamps() -> list[tuple[float, str, str]]:
    """Simulate the four LED4 command patterns without sleeping."""

    decoder = TwoLedCommandDecoder()
    rows: list[tuple[float, str, str]] = []
    t = 0.0
    step_s = 0.05
    for state in [FOLLOW, SAFE, HOLD, FINISH]:
        segment_end = t + 2.0
        while t < segment_end:
            decoded = decoder.update(
                anchor_visible=True,
                signal_visible=signal_on_for_state(state, t),
                now=t,
            )
            rows.append((round(t, 2), state, decoded))
            t += step_s
    return rows


if __name__ == '__main__':
    for timestamp, commanded, decoded in demo_generated_timestamps():
        print(f'{timestamp:5.2f}s commanded={commanded:7s} decoded={decoded}')
