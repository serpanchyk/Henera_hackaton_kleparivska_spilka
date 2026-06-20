import asyncio
import importlib.util
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from drone_sdk.ros_node import CameraFramePacket, DroneROSNode

ROOT = Path(__file__).resolve().parents[1]
PROVIDER_PATH = ROOT / 'resources' / 'scripts' / 'cv_vision_provider.py'


def _load_provider_module():
    spec = importlib.util.spec_from_file_location(
        'cv_vision_provider_under_test',
        PROVIDER_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


provider_module = _load_provider_module()
CVVisionProvider = provider_module.CVVisionProvider


class _Bridge:
    def imgmsg_to_cv2(self, message, desired_encoding):
        return message.image


class _FakeDrone:
    def __init__(self, packet):
        self.packet = packet
        self.spin_count = 0

    def spin(self):
        self.spin_count += 1

    def camera_frame_with_metadata(self):
        return self.packet

    def camera_diagnostics(self):
        sequence = 0 if self.packet is None else self.packet.sequence
        return {
            'camera_callback_count': sequence,
            'latest_frame_sequence': sequence,
        }


def _message(image, sec=3, nanosec=500):
    return SimpleNamespace(
        image=image,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
    )


def _packet(sequence, timestamp):
    return CameraFramePacket(
        image=np.zeros((20, 20, 3), dtype=np.uint8),
        sequence=sequence,
        timestamp=timestamp,
        ros_timestamp=123.0,
    )


class CameraPacketTests(unittest.TestCase):
    def test_callbacks_assign_monotonic_sequences(self):
        node = object.__new__(DroneROSNode)
        node._bridge = _Bridge()
        node._lock = threading.Lock()
        node._latest_packet = None
        node._camera_callback_count = 0
        node._latest_frame_sequence = 0

        DroneROSNode._image_cb(node, _message(np.zeros((2, 2, 3), dtype=np.uint8)))
        first = DroneROSNode.frame_with_metadata(node)
        DroneROSNode._image_cb(node, _message(np.ones((2, 2, 3), dtype=np.uint8)))
        second = DroneROSNode.frame_with_metadata(node)

        self.assertEqual(first.sequence, 1)
        self.assertEqual(second.sequence, 2)
        self.assertEqual(DroneROSNode.camera_diagnostics(node)['camera_callback_count'], 2)
        self.assertEqual(second.ros_timestamp, 3.0000005)


class CVFrameDeduplicationTests(unittest.TestCase):
    def setUp(self):
        self.drone = _FakeDrone(_packet(1, 10.0))
        self.provider = CVVisionProvider(self.drone)
        self.detect_calls = 0

        def detect(*_args, **_kwargs):
            self.detect_calls += 1
            return (10.0, 10.0, 12.0)

        self.detect_patch = patch.object(provider_module, '_detect_color', side_effect=detect)
        self.detect_patch.start()
        self.addCleanup(self.detect_patch.stop)

    def observe(self):
        return asyncio.run(self.provider.observe())

    def test_duplicate_reads_add_one_sample_and_return_cache(self):
        first = self.observe()
        decoder_before = self.provider.decoder.debug_stats()
        for _ in range(4):
            duplicate = self.observe()
            self.assertIs(duplicate, first)

        decoder_after = self.provider.decoder.debug_stats()
        self.assertEqual(self.detect_calls, 2)
        self.assertEqual(self.provider.decoder_sample_count, 1)
        self.assertEqual(self.provider.unique_processed_frames, 1)
        self.assertEqual(self.provider.duplicate_observe_frames, 4)
        self.assertEqual(decoder_before['transition_count'], decoder_after['transition_count'])
        self.assertEqual(decoder_before['signal_on_ratio'], decoder_after['signal_on_ratio'])
        self.assertTrue(self.provider.last_debug['transition']['duplicate'])
        self.assertFalse(self.provider.last_debug['transition']['decoder_sample_added'])

    def test_two_unique_frames_add_two_samples_and_preserve_timestamp(self):
        first = self.observe()
        self.drone.packet = _packet(2, 10.1)
        second = self.observe()

        self.assertIsNot(first, second)
        self.assertEqual(self.provider.decoder_sample_count, 2)
        self.assertEqual(self.provider.unique_processed_frames, 2)
        self.assertEqual(self.provider.decoder.samples[-1].t, 10.1)
        self.assertEqual(second.timestamp, 10.1)
        self.assertEqual(self.provider.last_debug['transition']['frame_sequence'], 2)
        self.assertEqual(self.provider.last_debug['transition']['frame_timestamp'], 10.1)
        self.assertTrue(self.provider.last_debug['transition']['decoder_sample_added'])


if __name__ == '__main__':
    unittest.main()
