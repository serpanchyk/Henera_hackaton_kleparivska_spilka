from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .types import LedBlob, TwoLedObservation


@dataclass(frozen=True)
class _PairCandidate:
    anchor: LedBlob
    signal: LedBlob
    distance_px: float
    confidence: float


class TwoLedTracker:
    """Track the leader midpoint from two visible LED blobs.

    This tracker is the green-only fallback used by the older debug detector.
    When blobs do not carry distinct color identity, the anchor is chosen
    deterministically as the leftmost blob, with topmost as a tie-breaker. The
    resulting vector is an image-only convention used for stable fallback when
    only one LED is visible.
    """

    def __init__(
        self,
        frame_width: int = 640,
        frame_height: int = 480,
        horizontal_fov_rad: float = 1.6,
        led_baseline_m: float = 0.1077,
        min_pair_distance_px: float = 8.0,
        max_pair_distance_px: float = 300.0,
        last_vector_ttl_s: float = 0.8,
    ) -> None:
        if frame_width <= 0:
            raise ValueError('frame_width must be positive')
        if frame_height <= 0:
            raise ValueError('frame_height must be positive')
        if horizontal_fov_rad <= 0.0 or horizontal_fov_rad >= math.pi:
            raise ValueError('horizontal_fov_rad must be in the open interval (0, pi)')
        if led_baseline_m <= 0.0:
            raise ValueError('led_baseline_m must be positive')
        if min_pair_distance_px < 0.0:
            raise ValueError('min_pair_distance_px must be non-negative')
        if max_pair_distance_px < min_pair_distance_px:
            raise ValueError('max_pair_distance_px must be greater than or equal to min_pair_distance_px')
        if last_vector_ttl_s < 0.0:
            raise ValueError('last_vector_ttl_s must be non-negative')

        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.horizontal_fov_rad = float(horizontal_fov_rad)
        self.led_baseline_m = float(led_baseline_m)
        self.min_pair_distance_px = float(min_pair_distance_px)
        self.max_pair_distance_px = float(max_pair_distance_px)
        self.last_vector_ttl_s = float(last_vector_ttl_s)

        self.focal_px = self.frame_width / (2.0 * math.tan(self.horizontal_fov_rad / 2.0))

        self.last_vec_x: float | None = None
        self.last_vec_y: float | None = None
        self.last_distance_px: float | None = None
        self.last_range_m: float | None = None
        self.last_vector_time: float | None = None
        self.last_seen_time: float | None = None

    def update(
        self,
        blobs: list[LedBlob],
        state: str,
        now: float | None = None,
    ) -> TwoLedObservation:
        timestamp = time.monotonic() if now is None else float(now)

        if len(blobs) >= 2:
            pair = self._select_pair(blobs)
            if pair is not None:
                return self._observation_from_pair(pair, state, timestamp)

        if len(blobs) == 1:
            return self._observation_from_anchor(blobs[0], state, timestamp)

        return self._invisible_observation(
            state=state,
            timestamp=timestamp,
            anchor_visible=False,
        )

    def _select_pair(self, blobs: list[LedBlob]) -> _PairCandidate | None:
        best: _PairCandidate | None = None

        for index, first in enumerate(blobs):
            for second in blobs[index + 1 :]:
                distance_px = math.hypot(second.cx - first.cx, second.cy - first.cy)
                if distance_px < self.min_pair_distance_px or distance_px > self.max_pair_distance_px:
                    continue

                anchor, signal = self._sort_anchor_signal(first, second)
                confidence = _clamp((first.confidence + second.confidence) * 0.5)
                candidate = _PairCandidate(
                    anchor=anchor,
                    signal=signal,
                    distance_px=distance_px,
                    confidence=confidence,
                )
                if best is None or self._is_better_pair(candidate, best):
                    best = candidate

        return best

    def _sort_anchor_signal(self, first: LedBlob, second: LedBlob) -> tuple[LedBlob, LedBlob]:
        if (first.cx, first.cy) <= (second.cx, second.cy):
            return first, second
        return second, first

    def _is_better_pair(self, candidate: _PairCandidate, best: _PairCandidate) -> bool:
        if not math.isclose(candidate.distance_px, best.distance_px):
            return candidate.distance_px > best.distance_px
        return candidate.confidence > best.confidence

    def _observation_from_pair(
        self,
        pair: _PairCandidate,
        state: str,
        timestamp: float,
    ) -> TwoLedObservation:
        vec_x = pair.signal.cx - pair.anchor.cx
        vec_y = pair.signal.cy - pair.anchor.cy
        mid_x = pair.anchor.cx + vec_x * 0.5
        mid_y = pair.anchor.cy + vec_y * 0.5
        range_m = self._estimate_range(pair.distance_px)

        self.last_vec_x = vec_x
        self.last_vec_y = vec_y
        self.last_distance_px = pair.distance_px
        self.last_range_m = range_m
        self.last_vector_time = timestamp
        self.last_seen_time = timestamp

        return TwoLedObservation(
            visible=True,
            x_error=self._x_error(mid_x),
            y_error=self._y_error(mid_y),
            led_distance_px=pair.distance_px,
            pair_angle_rad=math.atan2(vec_y, vec_x),
            estimated_range_m=range_m,
            anchor_visible=True,
            signal_visible=True,
            state=state,
            confidence=pair.confidence,
            last_seen_age_s=0.0,
        )

    def _observation_from_anchor(
        self,
        anchor: LedBlob,
        state: str,
        timestamp: float,
    ) -> TwoLedObservation:
        vector_age = self._last_vector_age(timestamp)
        if (
            self.last_vec_x is None
            or self.last_vec_y is None
            or vector_age is None
            or vector_age > self.last_vector_ttl_s
        ):
            return self._invisible_observation(
                state=state,
                timestamp=timestamp,
                anchor_visible=True,
            )

        mid_x = anchor.cx + self.last_vec_x * 0.5
        mid_y = anchor.cy + self.last_vec_y * 0.5
        age_ratio = vector_age / self.last_vector_ttl_s if self.last_vector_ttl_s > 0.0 else 1.0
        confidence = _clamp(anchor.confidence * 0.5 * (1.0 - 0.5 * age_ratio))
        self.last_seen_time = timestamp

        return TwoLedObservation(
            visible=True,
            x_error=self._x_error(mid_x),
            y_error=self._y_error(mid_y),
            led_distance_px=self.last_distance_px,
            pair_angle_rad=math.atan2(self.last_vec_y, self.last_vec_x),
            estimated_range_m=self.last_range_m,
            anchor_visible=True,
            signal_visible=False,
            state=state,
            confidence=confidence,
            last_seen_age_s=0.0,
        )

    def _invisible_observation(
        self,
        state: str,
        timestamp: float,
        anchor_visible: bool,
    ) -> TwoLedObservation:
        return TwoLedObservation(
            visible=False,
            x_error=None,
            y_error=None,
            led_distance_px=None,
            pair_angle_rad=None,
            estimated_range_m=None,
            anchor_visible=anchor_visible,
            signal_visible=False,
            state=state,
            confidence=0.0,
            last_seen_age_s=self._last_seen_age(timestamp),
        )

    def _estimate_range(self, distance_px: float) -> float | None:
        if distance_px <= 0.0:
            return None
        return self.led_baseline_m * self.focal_px / distance_px

    def _x_error(self, x: float) -> float:
        return (x - self.frame_width * 0.5) / (self.frame_width * 0.5)

    def _y_error(self, y: float) -> float:
        return (y - self.frame_height * 0.5) / (self.frame_height * 0.5)

    def _last_vector_age(self, timestamp: float) -> float | None:
        if self.last_vector_time is None:
            return None
        return max(0.0, timestamp - self.last_vector_time)

    def _last_seen_age(self, timestamp: float) -> float:
        if self.last_seen_time is None:
            return 0.0
        return max(0.0, timestamp - self.last_seen_time)


def demo_fake_blob_tracking() -> tuple[TwoLedObservation, TwoLedObservation, TwoLedObservation]:
    """Run a tiny fake-blob tracker demo for tests and manual sanity checks."""

    tracker = TwoLedTracker(frame_width=640, frame_height=480)
    pair_observation = tracker.update(
        [
            LedBlob(cx=300.0, cy=230.0, area=80.0, confidence=0.9),
            LedBlob(cx=360.0, cy=250.0, area=75.0, confidence=0.8),
        ],
        state='FOLLOW',
        now=10.0,
    )
    anchor_only_observation = tracker.update(
        [LedBlob(cx=302.0, cy=231.0, area=82.0, confidence=0.85)],
        state='SAFE',
        now=10.2,
    )
    no_blob_observation = tracker.update([], state='SAFE', now=10.5)

    return pair_observation, anchor_only_observation, no_blob_observation


def _clamp(value: float, minimum: float = 0.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, float(value)))
