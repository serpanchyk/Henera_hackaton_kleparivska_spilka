#!/usr/bin/env python3
"""Camera observation source for the two-LED follower pipeline."""
import math
import time
from typing import Optional

import cv2
import numpy as np

from drone_sdk.follower_controller import VisualObservation
from drone_sdk.two_led_cv import TwoLedCommandDecoder, two_led_to_visual
from drone_sdk.two_led_cv.types import TwoLedObservation

HFOV_RAD = 1.6
LED_BASELINE_M = 0.1077
GREEN_RANGES = [((40, 60, 60), (90, 255, 255))]
RED_RANGES = [((0, 70, 70), (14, 255, 255)), ((166, 70, 70), (180, 255, 255))]
_MIN_LED_AREA = 1.5
_RED_MIN_AREA = 1.5
_MIN_PAIR_RANGE_M = 0.3
_MAX_PAIR_RANGE_M = 30.0
_VISIBILITY_DEBOUNCE_FRAMES = 2
_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


class _VisibilityDebouncer:
    def __init__(self, frames: int = _VISIBILITY_DEBOUNCE_FRAMES):
        self.frames = max(1, int(frames))
        self._stable = False
        self._pending = False
        self._count = 0
        self._seeded = False

    def update(self, raw: bool) -> bool:
        raw = bool(raw)
        if not self._seeded:
            self._stable = raw
            self._pending = raw
            self._seeded = True
            return self._stable
        if raw == self._stable:
            self._pending = raw
            self._count = 0
            return self._stable
        if raw == self._pending:
            self._count += 1
        else:
            self._pending = raw
            self._count = 1
        if self._count >= self.frames:
            self._stable = raw
            self._count = 0
        return self._stable


def _detect_color(hsv, ranges, min_area=3.0, max_area=4000.0):
    mask = None
    for lo, hi in ranges:
        color_mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = color_mask if mask is None else cv2.bitwise_or(mask, color_mask)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area or area > max_area:
            continue
        moments = cv2.moments(contour)
        if moments['m00'] == 0.0:
            continue
        candidate = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'], area)
        if best is None or candidate[2] > best[2]:
            best = candidate
    return best


def _estimated_pair_range_m(anchor, signal, width: int, hfov_rad: float) -> Optional[float]:
    if anchor is None or signal is None:
        return None
    distance_px = math.hypot(signal[0] - anchor[0], signal[1] - anchor[1])
    if distance_px <= 0.0:
        return None
    focal_px = (width * 0.5) / math.tan(hfov_rad * 0.5)
    return LED_BASELINE_M * focal_px / distance_px


def _valid_led_pair(anchor, signal, width: int, hfov_rad: float) -> bool:
    estimated_range = _estimated_pair_range_m(anchor, signal, width, hfov_rad)
    return (
        estimated_range is not None
        and _MIN_PAIR_RANGE_M <= estimated_range <= _MAX_PAIR_RANGE_M
    )


class CVVisionProvider:
    def __init__(self, drone, hfov_rad: float = HFOV_RAD, show: bool = False,
                 window_name: str = None):
        self.drone = drone
        self.hfov_rad = hfov_rad
        self.decoder = TwoLedCommandDecoder()
        self._anchor_debounce = _VisibilityDebouncer()
        self._signal_debounce = _VisibilityDebouncer()
        self.show = show
        self.window_name = window_name or f'Follower {getattr(drone, "drone_id", "?")}'
        self.last_debug = {}

        self.last_processed_frame_sequence: Optional[int] = None
        self._cached_observation: Optional[VisualObservation] = None
        self.observe_call_count = 0
        self.unique_processed_frames = 0
        self.duplicate_observe_frames = 0
        self.decoder_sample_count = 0
        self._first_unique_frame_timestamp: Optional[float] = None
        self._last_unique_frame_timestamp: Optional[float] = None

        if self.show:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    async def observe(self) -> VisualObservation:
        self.observe_call_count += 1
        self.drone.spin()
        packet = self.drone.camera_frame_with_metadata()
        diagnostics = self.drone.camera_diagnostics()

        if packet is None:
            visual = VisualObservation(False, 0.0, 0.0, 0.0, 'UNKNOWN', time.monotonic())
            self._set_debug(
                diagnostics=diagnostics,
                frame_sequence=None,
                frame_timestamp=None,
                anchor_visible=False,
                signal_visible=False,
                decoded_state='UNKNOWN',
                estimated_range_m=None,
                led_distance_px=None,
                target_size=visual.target_size,
                decoder_sample_added=False,
                duplicate=False,
            )
            return visual

        if packet.sequence == self.last_processed_frame_sequence:
            self.duplicate_observe_frames += 1
            if self._cached_observation is None:
                raise RuntimeError('duplicate frame without a cached observation')
            cached = self._cached_observation
            self._set_debug(
                diagnostics=diagnostics,
                frame_sequence=packet.sequence,
                frame_timestamp=packet.timestamp,
                anchor_visible=self.last_debug.get('anchor_visible', False),
                signal_visible=self.last_debug.get('signal_visible', False),
                decoded_state=cached.mission_state,
                estimated_range_m=self.last_debug.get('estimated_range_m'),
                led_distance_px=self.last_debug.get('led_distance_px'),
                target_size=cached.target_size,
                decoder_sample_added=False,
                duplicate=True,
            )
            return cached

        self.last_processed_frame_sequence = packet.sequence
        self.unique_processed_frames += 1
        self.decoder_sample_count += 1
        if self._first_unique_frame_timestamp is None:
            self._first_unique_frame_timestamp = packet.timestamp
        self._last_unique_frame_timestamp = packet.timestamp

        frame = packet.image
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        anchor = _detect_color(hsv, GREEN_RANGES, min_area=_MIN_LED_AREA)
        signal = _detect_color(hsv, RED_RANGES, min_area=_RED_MIN_AREA)
        if anchor is not None and signal is not None and not _valid_led_pair(
            anchor, signal, width, self.hfov_rad
        ):
            signal = None

        anchor_visible = self._anchor_debounce.update(anchor is not None)
        signal_visible = self._signal_debounce.update(signal is not None)
        state = self.decoder.update(
            anchor_visible=anchor_visible,
            signal_visible=signal_visible,
            now=packet.timestamp,
        )

        observation = self._build_observation(anchor, signal, state, width, height)
        visual = two_led_to_visual(
            observation,
            timestamp=packet.timestamp,
            horizontal_fov_rad=self.hfov_rad,
            frame_aspect=height / width,
        )
        self._cached_observation = visual
        self._set_debug(
            diagnostics=diagnostics,
            frame_sequence=packet.sequence,
            frame_timestamp=packet.timestamp,
            anchor_visible=observation.anchor_visible,
            signal_visible=observation.signal_visible,
            decoded_state=state,
            estimated_range_m=observation.estimated_range_m,
            led_distance_px=observation.led_distance_px,
            target_size=visual.target_size,
            decoder_sample_added=True,
            duplicate=False,
        )

        if self.show:
            self._draw(frame, anchor, signal, visual)
        return visual

    def _set_debug(
        self,
        *,
        diagnostics,
        frame_sequence,
        frame_timestamp,
        anchor_visible,
        signal_visible,
        decoded_state,
        estimated_range_m,
        led_distance_px,
        target_size,
        decoder_sample_added,
        duplicate,
    ) -> None:
        elapsed = 0.0
        if (
            self._first_unique_frame_timestamp is not None
            and self._last_unique_frame_timestamp is not None
        ):
            elapsed = max(
                0.0,
                self._last_unique_frame_timestamp - self._first_unique_frame_timestamp,
            )
        rate = self.unique_processed_frames / elapsed if elapsed > 0.0 else 0.0
        decoder_stats = self.decoder.debug_stats() if self.decoder_sample_count else {}
        self.last_debug = {
            'camera_callback_count': diagnostics.get('camera_callback_count', 0),
            'latest_frame_sequence': diagnostics.get('latest_frame_sequence', 0),
            'observe_call_count': self.observe_call_count,
            'unique_processed_frames': self.unique_processed_frames,
            'duplicate_observe_frames': self.duplicate_observe_frames,
            'decoder_sample_count': self.decoder_sample_count,
            'effective_unique_frame_rate': rate,
            'effective_decoder_sample_rate': rate,
            'anchor_visible': anchor_visible,
            'signal_visible': signal_visible,
            'decoded_state': decoded_state,
            'estimated_range_m': estimated_range_m,
            'led_distance_px': led_distance_px,
            'target_size': target_size,
            'transition': {
                'frame_sequence': frame_sequence,
                'frame_timestamp': frame_timestamp,
                'anchor_visible': anchor_visible,
                'signal_visible': signal_visible,
                'decoder_sample_added': decoder_sample_added,
                'duplicate': duplicate,
                'decoded_state': decoded_state,
            },
            **decoder_stats,
        }

    def _build_observation(self, anchor, signal, state, width, height):
        if anchor is not None and signal is not None:
            target_x = (anchor[0] + signal[0]) * 0.5
            target_y = (anchor[1] + signal[1]) * 0.5
            distance_px = math.hypot(signal[0] - anchor[0], signal[1] - anchor[1])
            focal_px = (width * 0.5) / math.tan(self.hfov_rad * 0.5)
            estimated_range = (
                LED_BASELINE_M * focal_px / distance_px if distance_px > 0.0 else None
            )
            visible = True
        elif anchor is not None:
            target_x, target_y = anchor[0], anchor[1]
            distance_px = None
            estimated_range = None
            visible = True
        else:
            return TwoLedObservation(
                visible=False, x_error=None, y_error=None, led_distance_px=None,
                pair_angle_rad=None, estimated_range_m=None, anchor_visible=False,
                signal_visible=signal is not None, state=state, confidence=0.0,
                last_seen_age_s=0.0,
            )

        return TwoLedObservation(
            visible=visible,
            x_error=(target_x - width * 0.5) / (width * 0.5),
            y_error=(target_y - height * 0.5) / (height * 0.5),
            led_distance_px=distance_px,
            pair_angle_rad=None,
            estimated_range_m=estimated_range,
            anchor_visible=anchor is not None,
            signal_visible=signal is not None,
            state=state,
            confidence=1.0,
            last_seen_age_s=0.0,
        )

    def _draw(self, frame, anchor, signal, visual):
        debug_frame = frame.copy()
        if anchor is not None:
            cv2.circle(debug_frame, (int(anchor[0]), int(anchor[1])), 8, (0, 255, 0), 2)
        if signal is not None:
            cv2.circle(debug_frame, (int(signal[0]), int(signal[1])), 8, (0, 0, 255), 2)
        text = (
            f'{visual.mission_state} vis={visual.target_visible} '
            f'h={visual.horizontal_angle_deg:.1f} v={visual.vertical_angle_deg:.1f} '
            f'sz={visual.target_size:.0f}'
        )
        cv2.putText(debug_frame, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(debug_frame, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(self.window_name, debug_frame)
        cv2.waitKey(1)

    async def __call__(self) -> VisualObservation:
        return await self.observe()
