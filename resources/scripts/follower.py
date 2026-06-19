#!/usr/bin/env python3
"""
follower.py --id N

Follower drone script for one drone (N = 1, 2, or 3).
Launched automatically by solution_launch.py.

Integration points:
  detect_led(frame)      → Person 2 fills this function
  compute_velocity(det)  → Person 3 fills this function
"""
import argparse
import asyncio
import os
import signal
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.dirname(__file__))

import cv2
import rclpy
from drone_sdk import Drone
from results_logger import ResultsLogger

TAKEOFF_ALT_M = 5.0
CONTROL_HZ = 10
DEFAULT_SPEED = 1.5  # m/s passed to drone.move()


# ─── Person 2: LED Detection ──────────────────────────────────────────────────

def detect_led(frame):
    """
    Detect LED beacon in a camera frame and decode the optical signal.

    Args:
        frame: OpenCV BGR image (numpy array), or None if camera not ready.

    Returns:
        dict with keys:
          visible (bool)   — True if LED found in frame
          x_error (float)  — horizontal offset [-1.0 left .. +1.0 right]
          y_error (float)  — vertical offset   [-1.0 up   .. +1.0 down]
          size    (int)    — LED bounding-box area in pixels (distance proxy)
          state   (str)    — decoded command: FOLLOW | HOLD | FINISH | UNKNOWN
    """
    # TODO: Person 2 — implement OpenCV LED detection here.
    # Suggested steps:
    #   1. Convert frame to HSV
    #   2. Threshold for green LED colour (hue ~60, high saturation/value)
    #   3. Find contours, filter by area
    #   4. Compute centroid → x_error = (cx - w/2) / (w/2)
    #   5. Decode blink pattern → state
    return {"visible": False, "x_error": 0.0, "y_error": 0.0, "size": 0, "state": "UNKNOWN"}


# ─── Person 3: Velocity Controller ───────────────────────────────────────────

def compute_velocity(detection):
    """
    Convert LED detection result into body-frame velocity commands.

    Args:
        detection (dict): output of detect_led()

    Returns:
        (forward, right, down) in m/s (body frame).
        Negative down = upward.
    """
    # TODO: Person 3 — implement controller here.
    # Suggested logic:
    #   TARGET_SIZE = 800       # pixels — desired distance from leader
    #   KP_YAW = 0.5            # proportional gain for lateral centering
    #   KP_DIST = 0.003         # proportional gain for distance keeping
    #
    #   if not detection["visible"]:  → SEARCH mode: slow yaw scan
    #   right   = KP_YAW  * detection["x_error"]
    #   forward = KP_DIST * (TARGET_SIZE - detection["size"])
    #   Clamp velocities to safe limits before returning.
    return 0.0, 0.0, 0.0  # forward, right, down


# ─── Camera thread ────────────────────────────────────────────────────────────

def camera_loop(drone: Drone, detection_box: list, lock: threading.Lock,
                shutdown: threading.Event):
    drone.start_camera()
    win = f"Follower {drone.drone_id}"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while not shutdown.is_set() and rclpy.ok():
        drone.spin()
        frame = drone.camera_frame()
        result = detect_led(frame)
        with lock:
            detection_box[0] = result

        if frame is not None:
            cv2.imshow(win, frame)
        cv2.waitKey(1)

    cv2.destroyWindow(win)
    drone.stop_camera()


# ─── Control loop ─────────────────────────────────────────────────────────────

async def control_loop(drone: Drone, detection_box: list, lock: threading.Lock,
                       logger: ResultsLogger, shutdown: asyncio.Event):
    interval = 1.0 / CONTROL_HZ
    while not shutdown.is_set():
        with lock:
            detection = dict(detection_box[0])

        state = detection.get("state", "UNKNOWN")

        if state == "FINISH":
            print(f"[drone {drone.drone_id}] FINISH — landing")
            shutdown.set()
            break

        if state == "HOLD" or not detection["visible"]:
            await drone.move(0.0, 0.0, 0.0, speed_m_s=DEFAULT_SPEED)
        else:
            forward, right, down = compute_velocity(detection)
            await drone.move(forward, right, down, speed_m_s=DEFAULT_SPEED)

        logger.log(drone.drone_id, detection)
        await asyncio.sleep(interval)


# ─── Entry point ─────────────────────────────────────────────────────────────

async def main(drone_id: int):
    rclpy.init()
    drone = Drone(drone_id=drone_id)
    logger = ResultsLogger()
    lock = threading.Lock()
    detection_box = [{"visible": False, "x_error": 0.0, "y_error": 0.0,
                      "size": 0, "state": "UNKNOWN"}]
    shutdown_thread = threading.Event()
    shutdown_async = asyncio.Event()

    def on_sig(*_):
        shutdown_thread.set()
        shutdown_async.set()

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    print(f"[drone {drone_id}] Connecting...")
    try:
        await drone.connect()
    except Exception as e:
        print(f"[drone {drone_id}] Connection failed: {e}")
        rclpy.shutdown()
        return

    print(f"[drone {drone_id}] Connected — starting camera")
    cam_thread = threading.Thread(
        target=camera_loop,
        args=(drone, detection_box, lock, shutdown_thread),
        daemon=True,
    )
    cam_thread.start()

    print(f"[drone {drone_id}] Waiting 15s for EKF to stabilize...")
    await asyncio.sleep(15)

    print(f"[drone {drone_id}] Arming and taking off to {TAKEOFF_ALT_M}m")
    await drone.arm()
    await drone.takeoff(TAKEOFF_ALT_M)
    await asyncio.sleep(5)
    await drone.start_offboard()

    print(f"[drone {drone_id}] Offboard active — entering control loop")
    await control_loop(drone, detection_box, lock, logger, shutdown_async)

    await drone.stop_offboard()
    await drone.land()
    await asyncio.sleep(10)
    await drone.disarm()

    shutdown_thread.set()
    cam_thread.join(timeout=5)
    logger.save()
    await drone.close()
    rclpy.shutdown()
    print(f"[drone {drone_id}] Done")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Follower drone controller")
    parser.add_argument("--id", type=int, required=True, help="Drone ID (1, 2, or 3)")
    args = parser.parse_args()
    asyncio.run(main(args.id))
