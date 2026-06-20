import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


CAMERA_TOPIC = '/world/baylands_custom/model/x500_mono_cam_{}/link/mono_cam/base_link/sensor/camera_sensor/image'
LED_TOPIC = '/model/x500_mono_cam_{}/led_cmd'


@dataclass(frozen=True)
class CameraFramePacket:
    """One successfully converted camera frame and its callback identity."""

    image: np.ndarray
    sequence: int
    timestamp: float
    ros_timestamp: Optional[float] = None


class DroneROSNode(Node):

    def __init__(self, drone_id: int):
        super().__init__(f'drone_{drone_id}_sdk_node')
        self._drone_id = drone_id
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_packet: Optional[CameraFramePacket] = None
        self._camera_callback_count = 0
        self._latest_frame_sequence = 0

        cam_topic = CAMERA_TOPIC.format(drone_id)
        self._cam_sub = self.create_subscription(
            Image, cam_topic, self._image_cb, 10
        )

        led_topic = LED_TOPIC.format(drone_id)
        self._led_pub = self.create_publisher(String, led_topic, 10)

        self._spin_thread: Optional[threading.Thread] = None
        self._spinning = False

    def _image_cb(self, msg: Image) -> None:
        # Keep the follower controller and CV samples in one monotonic time base.
        timestamp = time.monotonic()
        ros_timestamp = _ros_header_timestamp(msg)
        with self._lock:
            self._camera_callback_count += 1
            self._latest_frame_sequence += 1
            sequence = self._latest_frame_sequence

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        with self._lock:
            self._latest_packet = CameraFramePacket(
                image=frame,
                sequence=sequence,
                timestamp=timestamp,
                ros_timestamp=ros_timestamp,
            )

    def frame(self) -> Optional[np.ndarray]:
        packet = self.frame_with_metadata()
        return None if packet is None else packet.image

    def frame_with_metadata(self) -> Optional[CameraFramePacket]:
        with self._lock:
            return self._latest_packet

    def camera_diagnostics(self) -> dict[str, int]:
        with self._lock:
            return {
                'camera_callback_count': self._camera_callback_count,
                'latest_frame_sequence': self._latest_frame_sequence,
            }

    def publish_led(self, value: str) -> None:
        msg = String()
        msg.data = value
        self._led_pub.publish(msg)

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.001)

    def start_spin(self) -> None:
        if self._spinning:
            return
        self._spinning = True
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def stop_spin(self) -> None:
        self._spinning = False

    def _spin_loop(self) -> None:
        while self._spinning and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)


def _ros_header_timestamp(msg: Image) -> Optional[float]:
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', None)

    if not isinstance(sec, int) or not isinstance(nanosec, int):
        return None
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None

    value = sec + nanosec / 1_000_000_000.0
    return value if math.isfinite(value) else None
