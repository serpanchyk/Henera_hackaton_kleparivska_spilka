#!/usr/bin/env python3
"""
CVVisionProvider — real camera-based observation source for the follower controller.

Two-colour optical channel: the leader's anchor LED (led_lens_01) is GREEN and is
always on; the signal LED (led_lens_04) is RED and blinks the command state. Using
distinct colours lets us identify anchor vs signal reliably even when the two LEDs
overlap in the image — green and red never merge into one ambiguous blob, which
removes the "only one green blob -> SAFE" failure mode.

Pipeline (no ground truth — legal submission path):
    frame -> green blobs (anchor) + red blobs (signal)
          -> decoder(anchor_visible, signal_visible) -> state
          -> TwoLedObservation -> two_led_to_visual() -> VisualObservation
"""
import math
import time

import cv2
import numpy as np

from drone_sdk.follower_controller import VisualObservation
from drone_sdk.two_led_cv import TwoLedCommandDecoder, two_led_to_visual
from drone_sdk.two_led_cv.types import TwoLedObservation

HFOV_RAD = 1.6
LED_BASELINE_M = 0.1077  # spacing between anchor and signal lenses on the model

# HSV ranges (OpenCV H in 0..179). Red wraps around 0, so two bands.
GREEN_RANGES = [((40, 60, 60), (90, 255, 255))]
RED_RANGES = [((0, 90, 90), (12, 255, 255)), ((168, 90, 90), (180, 255, 255))]

_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


def _detect_color(hsv, ranges, min_area=4.0, max_area=4000.0):
    """Return (cx, cy, area) of the largest blob matching any HSV range, or None."""
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < min_area or area > max_area:
            continue
        mom = cv2.moments(c)
        if mom['m00'] == 0.0:
            continue
        cx = mom['m10'] / mom['m00']
        cy = mom['m01'] / mom['m00']
        if best is None or area > best[2]:
            best = (cx, cy, area)
    return best


class CVVisionProvider:
    def __init__(self, drone, hfov_rad: float = HFOV_RAD, show: bool = False,
                 window_name: str = None):
        self.drone = drone
        self.hfov_rad = hfov_rad
        self.decoder = TwoLedCommandDecoder()
        self.show = show
        self.window_name = window_name or f'Follower {getattr(drone, "drone_id", "?")}'
        self.last_debug = {}
        if self.show:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    async def observe(self) -> VisualObservation:
        self.drone.spin()
        frame = self.drone.camera_frame()
        if frame is None:
            return VisualObservation(False, 0.0, 0.0, 0.0, 'UNKNOWN', time.monotonic())

        now = time.monotonic()
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        anchor = _detect_color(hsv, GREEN_RANGES)   # green = anchor
        signal = _detect_color(hsv, RED_RANGES)     # red = signal

        anchor_visible = anchor is not None
        signal_visible = signal is not None
        state = self.decoder.update(anchor_visible=anchor_visible,
                                    signal_visible=signal_visible, now=now)

        obs = self._build_observation(anchor, signal, state, width, height)
        visual = two_led_to_visual(obs, horizontal_fov_rad=self.hfov_rad,
                                   frame_aspect=height / width)
        self.last_debug = {
            'anchor_visible': obs.anchor_visible,
            'signal_visible': obs.signal_visible,
            'decoder_state': state,
            'estimated_range_m': obs.estimated_range_m,
            'led_distance_px': obs.led_distance_px,
            **self.decoder.debug_stats(),
        }

        if self.show:
            self._draw(frame, anchor, signal, visual)
        return visual

    def _build_observation(self, anchor, signal, state, width, height):
        # Target point: midpoint of anchor+signal if both, else anchor alone.
        if anchor is not None and signal is not None:
            tx = (anchor[0] + signal[0]) * 0.5
            ty = (anchor[1] + signal[1]) * 0.5
            dist_px = math.hypot(signal[0] - anchor[0], signal[1] - anchor[1])
            focal_px = (width * 0.5) / math.tan(self.hfov_rad * 0.5)
            est_range = LED_BASELINE_M * focal_px / dist_px if dist_px > 0 else None
            visible = True
        elif anchor is not None:
            tx, ty = anchor[0], anchor[1]
            dist_px = None
            est_range = None
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
            x_error=(tx - width * 0.5) / (width * 0.5),
            y_error=(ty - height * 0.5) / (height * 0.5),
            led_distance_px=dist_px,
            pair_angle_rad=None,
            estimated_range_m=est_range,
            anchor_visible=anchor is not None,
            signal_visible=signal is not None,
            state=state,
            confidence=1.0,
            last_seen_age_s=0.0,
        )

    def _draw(self, frame, anchor, signal, visual):
        dbg = frame.copy()
        if anchor is not None:
            cv2.circle(dbg, (int(anchor[0]), int(anchor[1])), 8, (0, 255, 0), 2)
        if signal is not None:
            cv2.circle(dbg, (int(signal[0]), int(signal[1])), 8, (0, 0, 255), 2)
        txt = (f'{visual.mission_state} vis={visual.target_visible} '
               f'h={visual.horizontal_angle_deg:.1f} v={visual.vertical_angle_deg:.1f} '
               f'sz={visual.target_size:.0f}')
        cv2.putText(dbg, txt, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(dbg, txt, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(self.window_name, dbg)
        cv2.waitKey(1)

    async def __call__(self) -> VisualObservation:
        return await self.observe()
