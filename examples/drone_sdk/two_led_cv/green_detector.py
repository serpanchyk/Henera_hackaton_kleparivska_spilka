from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np

from .types import LedBlob


class GreenLedDetector:
    """Detect green LED-like blobs in a BGR camera frame."""

    def __init__(
        self,
        hsv_lower: tuple[int, int, int] = (40, 60, 60),
        hsv_upper: tuple[int, int, int] = (90, 255, 255),
        min_area: float = 4.0,
        max_area: float = 2500.0,
        min_circularity: float = 0.45,
        min_aspect_ratio: float = 0.45,
        max_aspect_ratio: float = 2.2,
        morph_kernel_size: int = 3,
        morph_iterations: int = 1,
    ) -> None:
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_area = min_area
        self.max_area = max_area
        self.min_circularity = min_circularity
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.morph_iterations = morph_iterations

        kernel_size = max(1, int(morph_kernel_size))
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

    def detect(self, frame_bgr: np.ndarray) -> list[LedBlob]:
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError('frame_bgr must be a BGR image with shape (height, width, 3)')

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=self.morph_iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=self.morph_iterations)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs: list[LedBlob] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area or area > self.max_area:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue

            circularity = float(4.0 * math.pi * area / (perimeter * perimeter))
            if circularity < self.min_circularity:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            if width <= 0 or height <= 0:
                continue

            aspect_ratio = width / height
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue

            moments = cv2.moments(contour)
            if moments['m00'] == 0.0:
                continue

            cx = float(moments['m10'] / moments['m00'])
            cy = float(moments['m01'] / moments['m00'])

            roi = hsv[y : y + height, x : x + width]
            roi_mask = mask[y : y + height, x : x + width]
            saturation = _masked_channel_mean(roi[:, :, 1], roi_mask) / 255.0
            brightness = _masked_channel_mean(roi[:, :, 2], roi_mask) / 255.0

            confidence = self._confidence(area, circularity, aspect_ratio, saturation, brightness)
            blobs.append(LedBlob(cx=cx, cy=cy, area=area, confidence=confidence))

        return sorted(blobs, key=lambda blob: blob.confidence, reverse=True)

    def _confidence(
        self,
        area: float,
        circularity: float,
        aspect_ratio: float,
        saturation: float,
        brightness: float,
    ) -> float:
        if self.max_area <= self.min_area:
            area_score = 1.0
        else:
            area_score = (area - self.min_area) / (self.max_area - self.min_area)
        area_score = _clamp(area_score)

        circularity_score = _clamp(circularity)
        aspect_score = _clamp(1.0 - abs(1.0 - aspect_ratio))
        color_score = _clamp((saturation + brightness) * 0.5)

        return _clamp(
            0.30 * area_score
            + 0.30 * circularity_score
            + 0.20 * aspect_score
            + 0.20 * color_score
        )


def draw_debug(
    frame_bgr: np.ndarray,
    blobs: list[LedBlob],
    observation: Any | None = None,
) -> np.ndarray:
    debug = frame_bgr.copy()

    for index, blob in enumerate(blobs):
        center = (int(round(blob.cx)), int(round(blob.cy)))
        radius = max(3, int(round(math.sqrt(blob.area / math.pi))))
        color = (0, 255, 0) if index == 0 else (0, 180, 255)
        cv2.circle(debug, center, radius, color, 2)
        cv2.drawMarker(debug, center, color, cv2.MARKER_CROSS, 12, 1)
        cv2.putText(
            debug,
            f'{blob.confidence:.2f}',
            (center[0] + 6, center[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            color,
            1,
            cv2.LINE_AA,
        )

    if observation is not None:
        _draw_observation(debug, observation)

    state = getattr(observation, 'state', 'NO_OBS') if observation is not None else 'BLOBS'
    visible = getattr(observation, 'visible', None) if observation is not None else None
    text = f'state={state} blobs={len(blobs)}'
    if visible is not None:
        text += f' visible={visible}'
    cv2.putText(debug, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(debug, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 1, cv2.LINE_AA)

    return debug


def _draw_observation(frame_bgr: np.ndarray, observation: Any) -> None:
    height, width = frame_bgr.shape[:2]
    x_error = getattr(observation, 'x_error', None)
    y_error = getattr(observation, 'y_error', None)
    if x_error is None or y_error is None:
        return

    midpoint = (
        int(round((x_error + 1.0) * 0.5 * width)),
        int(round((y_error + 1.0) * 0.5 * height)),
    )
    cv2.drawMarker(frame_bgr, midpoint, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
    cv2.circle(frame_bgr, midpoint, 5, (255, 0, 255), 2)


def _masked_channel_mean(channel: np.ndarray, mask: np.ndarray) -> float:
    mean = cv2.mean(channel, mask=mask)[0]
    return float(mean)


def _clamp(value: float, minimum: float = 0.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, float(value)))
