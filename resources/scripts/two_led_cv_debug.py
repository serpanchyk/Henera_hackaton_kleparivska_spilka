#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from drone_sdk.two_led_cv import GreenLedDetector, TwoLedCommandDecoder, TwoLedTracker  # noqa: E402
from drone_sdk.two_led_cv.types import LedBlob, TwoLedObservation  # noqa: E402

DEFAULT_CAMERA_TOPIC = (
    '/world/baylands_custom/model/x500_mono_cam_1/link/mono_cam/base_link/sensor/'
    'camera_sensor/image'
)
WINDOW_NAME = 'two_led_cv_debug'
MASK_WINDOW_NAME = 'two_led_cv_mask'
RED_RANGES = [((0, 70, 70), (14, 255, 255)), ((166, 70, 70), (180, 255, 255))]
RED_MIN_AREA = 3.0


class FrameSource(Protocol):
    def read(self) -> np.ndarray | None:
        ...

    def close(self) -> None:
        ...


class OpenCvFrameSource:
    def __init__(self, source: str) -> None:
        self.image: np.ndarray | None = None
        self.capture: cv2.VideoCapture | None = None

        path = Path(source).expanduser()
        if path.exists() and path.is_file() and path.suffix.lower() in {
            '.bmp',
            '.jpg',
            '.jpeg',
            '.png',
            '.tif',
            '.tiff',
            '.webp',
        }:
            self.image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if self.image is None:
                raise RuntimeError(f'failed to read image: {path}')
            return

        capture_source: int | str
        if source.isdigit():
            capture_source = int(source)
        else:
            capture_source = str(path if path.exists() else source)

        self.capture = cv2.VideoCapture(capture_source)
        if not self.capture.isOpened():
            raise RuntimeError(f'failed to open video source: {source}')

    def read(self) -> np.ndarray | None:
        if self.image is not None:
            return self.image.copy()
        assert self.capture is not None
        ok, frame = self.capture.read()
        return frame if ok else None

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()


class RosImageFrameSource:
    def __init__(self, topic: str) -> None:
        try:
            import rclpy
            from cv_bridge import CvBridge
            from rclpy.node import Node
            from sensor_msgs.msg import Image
        except ImportError as exc:
            raise RuntimeError(
                'ROS image source requires ROS 2, sensor_msgs, and cv_bridge. '
                'Run `source /opt/ros/humble/setup.bash` first.'
            ) from exc

        self.rclpy = rclpy
        self.bridge = CvBridge()
        self.frame: np.ndarray | None = None
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        self.node = Node('two_led_cv_debug')
        self.subscription = self.node.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg) -> None:
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def read(self) -> np.ndarray | None:
        self.rclpy.spin_once(self.node, timeout_sec=0.01)
        return None if self.frame is None else self.frame.copy()

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy:
            self.rclpy.shutdown()


def main() -> int:
    args = parse_args()
    source = make_source(args.source, args.ros)
    detector = GreenLedDetector(
        hsv_lower=tuple(args.hsv_lower),
        hsv_upper=tuple(args.hsv_upper),
        min_area=args.min_area,
        max_area=args.max_area,
    )
    decoder = TwoLedCommandDecoder(window_s=args.decoder_window_s, min_samples=args.decoder_min_samples)
    tracker: TwoLedTracker | None = None
    last_print_s = 0.0

    if args.debug:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        if args.show_mask:
            cv2.namedWindow(MASK_WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = source.read()
            if frame is None:
                if isinstance(source, RosImageFrameSource):
                    if args.debug:
                        key = cv2.waitKey(1) & 0xFF
                        if key in (27, ord('q')):
                            break
                    continue
                break

            height, width = frame.shape[:2]
            if tracker is None or tracker.frame_width != width or tracker.frame_height != height:
                tracker = TwoLedTracker(frame_width=width, frame_height=height)

            now = time.monotonic()
            green_blobs = detector.detect(frame)
            red_blob = detect_red_blob(frame)
            tracker_blobs = blobs_for_tracker(green_blobs, red_blob)
            green_visible = bool(green_blobs)
            red_visible = red_blob is not None
            state = decoder.update(anchor_visible=green_visible, signal_visible=red_visible, now=now)
            observation = tracker.update(tracker_blobs, state=state, now=now)
            stats = decoder.debug_stats()

            if now - last_print_s >= args.print_period:
                print_observation(observation, stats)
                last_print_s = now

            if args.debug:
                mask, contours = build_green_mask(frame, detector)
                debug = draw_overlay(frame, tracker_blobs, observation, tracker, stats, contours)
                cv2.imshow(WINDOW_NAME, debug)
                if args.show_mask:
                    cv2.imshow(MASK_WINDOW_NAME, mask)
                key = cv2.waitKey(args.wait_ms) & 0xFF
                if key in (27, ord('q')):
                    break

            if isinstance(source, OpenCvFrameSource) and source.image is not None:
                if not args.debug:
                    break
                cv2.waitKey(0)
                break
    finally:
        source.close()
        if args.debug:
            cv2.destroyAllWindows()

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run the two-LED detector, decoder, and tracker on camera/video/image frames.'
    )
    parser.add_argument(
        '--source',
        default=DEFAULT_CAMERA_TOPIC,
        help=(
            'Input source. Use `ros:/topic/name` for a ROS image topic, a plain topic path for '
            'the default ROS mode, a webcam index like `0`, or a video/image file path.'
        ),
    )
    parser.add_argument(
        '--ros',
        action='store_true',
        help='Force --source to be interpreted as a ROS sensor_msgs/Image topic.',
    )
    parser.add_argument('--debug', action='store_true', help='Show OpenCV debug windows.')
    parser.add_argument('--show-mask', action='store_true', help='Show the green threshold mask window.')
    parser.add_argument('--print-period', type=float, default=0.5, help='Observation print interval in seconds.')
    parser.add_argument('--wait-ms', type=int, default=1, help='OpenCV waitKey delay for video/debug mode.')
    parser.add_argument('--min-area', type=float, default=4.0, help='Minimum green blob contour area in px.')
    parser.add_argument('--max-area', type=float, default=2500.0, help='Maximum green blob contour area in px.')
    parser.add_argument('--hsv-lower', nargs=3, type=int, default=(40, 60, 60), metavar=('H', 'S', 'V'))
    parser.add_argument('--hsv-upper', nargs=3, type=int, default=(90, 255, 255), metavar=('H', 'S', 'V'))
    parser.add_argument('--decoder-window-s', type=float, default=2.0)
    parser.add_argument('--decoder-min-samples', type=int, default=10)
    return parser.parse_args()


def make_source(source: str, force_ros: bool) -> FrameSource:
    if source.startswith('ros:'):
        return RosImageFrameSource(source.removeprefix('ros:'))

    path = Path(source).expanduser()
    if not force_ros and (source.isdigit() or path.exists()):
        return OpenCvFrameSource(source)

    return RosImageFrameSource(source)


def blobs_for_tracker(green_blobs: list[LedBlob], red_blob: LedBlob | None) -> list[LedBlob]:
    blobs: list[LedBlob] = []
    if green_blobs:
        blobs.append(green_blobs[0])
    if red_blob is not None:
        blobs.append(red_blob)
    return blobs


def detect_red_blob(frame_bgr: np.ndarray) -> LedBlob | None:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in RED_RANGES:
        color_mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
        mask = color_mask if mask is None else cv2.bitwise_or(mask, color_mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best: LedBlob | None = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < RED_MIN_AREA:
            continue
        moments = cv2.moments(contour)
        if moments['m00'] == 0.0:
            continue
        candidate = LedBlob(
            cx=float(moments['m10'] / moments['m00']),
            cy=float(moments['m01'] / moments['m00']),
            area=area,
            confidence=1.0,
        )
        if best is None or candidate.area > best.area:
            best = candidate
    return best


def build_green_mask(frame_bgr: np.ndarray, detector: GreenLedDetector) -> tuple[np.ndarray, list[np.ndarray]]:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, detector.hsv_lower, detector.hsv_upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, detector.kernel, iterations=detector.morph_iterations)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, detector.kernel, iterations=detector.morph_iterations)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return mask, contours


def draw_overlay(
    frame_bgr: np.ndarray,
    blobs: list[LedBlob],
    observation: TwoLedObservation,
    tracker: TwoLedTracker,
    stats: dict[str, float | int | str],
    contours: list[np.ndarray],
) -> np.ndarray:
    debug = frame_bgr.copy()
    cv2.drawContours(debug, contours, -1, (0, 120, 0), 1)

    for index, blob in enumerate(blobs):
        center = point(blob.cx, blob.cy)
        radius = max(3, int(round(math.sqrt(blob.area / math.pi))))
        color = (0, 255, 0) if index == 0 else (0, 0, 255)
        cv2.circle(debug, center, radius, color, 2)
        cv2.drawMarker(debug, center, color, cv2.MARKER_CROSS, 12, 1)
        cv2.putText(debug, f'{index}:{blob.confidence:.2f}', (center[0] + 6, center[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    pair = tracker._select_pair(blobs) if len(blobs) >= 2 else None
    if pair is not None:
        anchor = point(pair.anchor.cx, pair.anchor.cy)
        signal = point(pair.signal.cx, pair.signal.cy)
        mid = point((pair.anchor.cx + pair.signal.cx) * 0.5, (pair.anchor.cy + pair.signal.cy) * 0.5)
        cv2.line(debug, anchor, signal, (255, 255, 0), 2)
        cv2.drawMarker(debug, mid, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
        cv2.circle(debug, mid, 5, (255, 0, 255), 2)
    elif len(blobs) == 1 and tracker.last_vec_x is not None and tracker.last_vec_y is not None:
        anchor_blob = blobs[0]
        estimated_mid = point(anchor_blob.cx + tracker.last_vec_x * 0.5, anchor_blob.cy + tracker.last_vec_y * 0.5)
        cv2.drawMarker(debug, estimated_mid, (255, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
        cv2.circle(debug, estimated_mid, 7, (255, 0, 255), 1)
        cv2.putText(debug, 'estimated midpoint', (estimated_mid[0] + 8, estimated_mid[1] + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 0, 255), 1, cv2.LINE_AA)

    draw_text_panel(debug, observation, stats)
    return debug


def draw_text_panel(
    frame_bgr: np.ndarray,
    observation: TwoLedObservation,
    stats: dict[str, float | int | str],
) -> None:
    lines = [
        f'state={observation.state} visible={observation.visible}',
        f'x_error={format_optional(observation.x_error)} y_error={format_optional(observation.y_error)}',
        f'led_distance_px={format_optional(observation.led_distance_px)}',
        'signal_on_ratio={:.2f} transitions_per_s={:.2f}'.format(
            float(stats['signal_on_ratio']),
            float(stats['transitions_per_s']),
        ),
    ]
    x, y = 10, 24
    for line in lines:
        cv2.putText(frame_bgr, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(frame_bgr, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
        y += 22


def print_observation(
    observation: TwoLedObservation,
    stats: dict[str, float | int | str],
) -> None:
    payload = asdict(observation)
    payload['decoder'] = {
        'signal_on_ratio': round(float(stats['signal_on_ratio']), 3),
        'transitions_per_s': round(float(stats['transitions_per_s']), 3),
        'sample_count': int(stats['sample_count']),
    }
    print(json.dumps(payload, sort_keys=True))


def point(x: float, y: float) -> tuple[int, int]:
    return int(round(x)), int(round(y))


def format_optional(value: float | None) -> str:
    return 'None' if value is None else f'{value:.3f}'


if __name__ == '__main__':
    raise SystemExit(main())
