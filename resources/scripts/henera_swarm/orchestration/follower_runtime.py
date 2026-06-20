#!/usr/bin/env python3
"""Production follower runtime for the official multi-process solution path."""

from __future__ import annotations

import argparse
import asyncio
import os
import signal
import sys
import time
from typing import Optional

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
_SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

from drone_sdk.follower_controller import (
    DroneFollowerActuator,
    FollowerCommand,
    FollowerController,
    FollowerControllerConfig,
    MissionState,
)
from drone_sdk.config import CONFIG
from drone_sdk.two_led_cv import mask_for_state
from henera_swarm.logging import ResultsLogger
from henera_swarm.perception import CVVisionProvider

TAKEOFF_ALT_M = CONFIG.runtime.common_alt_m
EKF_SETTLE_S = CONFIG.runtime.ekf_settle_s
TAKEOFF_SETTLE_S = 5.0
FINISH_DESCENT_SPEED_MS = CONFIG.runtime.finish_descent_speed_ms
FINISH_DESCENT_TIMEOUT_S = CONFIG.runtime.finish_descent_timeout_s
FINISH_GROUND_ALT_M = CONFIG.runtime.finish_ground_alt_m


def _state_str(value) -> str:
    return getattr(value, 'value', str(value))


def _target_id_for(drone_id: int) -> str:
    return 'leader' if drone_id == 1 else f'follower_{drone_id - 1}'


def _log_payload(command: FollowerCommand, provider: CVVisionProvider) -> dict:
    debug = dict(getattr(provider, 'last_debug', {}))
    payload = {
        'visible': bool(debug.get('anchor_visible', False)),
        'state': _state_str(command.relay_state),
        'controller_state': _state_str(command.state),
        'forward_m_s': command.forward_m_s,
        'right_m_s': command.right_m_s,
        'down_m_s': command.down_m_s,
        'yaw_rate_deg_s': command.yaw_rate_deg_s,
        'horizontal_angle_deg': command.horizontal_angle_deg,
        'vertical_angle_deg': command.vertical_angle_deg,
        'size_error': command.size_error,
    }
    payload.update(debug)
    return payload


async def _descend_after_finish(drone: Drone, actuator: DroneFollowerActuator) -> None:
    deadline = time.monotonic() + FINISH_DESCENT_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            position = await drone.position_ned()
            if -position.down_m <= FINISH_GROUND_ALT_M:
                return
            heading = await drone.heading()
            await drone.set_velocity(0.0, 0.0, FINISH_DESCENT_SPEED_MS, yaw_deg=heading)
        except Exception:
            await actuator.safe_stop()
            raise
        await asyncio.sleep(0.2)


async def _control_loop(
    *,
    drone: Drone,
    controller: FollowerController,
    provider: CVVisionProvider,
    actuator: DroneFollowerActuator,
    logger: ResultsLogger,
    shutdown: asyncio.Event,
) -> None:
    period = 1.0 / controller.config.control_rate_hz
    led_t = 0.0
    while not shutdown.is_set():
        observation = await provider.observe()
        command = controller.update(observation)

        relay_state = _state_str(command.relay_state)
        drone.set_leds(mask_for_state(relay_state, led_t))
        led_t += period

        logger.log(drone.drone_id, _log_payload(command, provider))

        if command.relay_state == MissionState.FINISH:
            print(f'[drone {drone.drone_id}] FINISH received — descending')
            await actuator.safe_stop()
            await _descend_after_finish(drone, actuator)
            shutdown.set()
            break

        await actuator.apply(command)
        await asyncio.sleep(period)


async def main(
    drone_id: int,
    *,
    show: bool = False,
    config: Optional[FollowerControllerConfig] = None,
) -> None:
    import rclpy
    from drone_sdk import Drone

    if not rclpy.ok():
        rclpy.init()

    drone = Drone(drone_id=drone_id)
    logger = ResultsLogger()
    shutdown = asyncio.Event()

    def on_signal(*_):
        shutdown.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    controller = FollowerController(
        follower_id=f'follower_{drone_id}',
        target_id=_target_id_for(drone_id),
        config=config or FollowerControllerConfig.stable(),
    )
    provider = CVVisionProvider(drone, show=show)
    actuator = DroneFollowerActuator(drone)

    print(f'[drone {drone_id}] Connecting...')
    try:
        await drone.connect()
        drone.start_camera()

        print(f'[drone {drone_id}] Waiting {EKF_SETTLE_S:.0f}s for EKF to stabilize...')
        await asyncio.sleep(EKF_SETTLE_S)

        print(f'[drone {drone_id}] Arming and taking off to {TAKEOFF_ALT_M}m')
        await drone.arm()
        await drone.takeoff(TAKEOFF_ALT_M)
        await asyncio.sleep(TAKEOFF_SETTLE_S)
        await drone.start_offboard()

        print(f'[drone {drone_id}] Offboard active — entering follower runtime')
        await _control_loop(
            drone=drone,
            controller=controller,
            provider=provider,
            actuator=actuator,
            logger=logger,
            shutdown=shutdown,
        )
    except Exception as exc:
        print(f'[drone {drone_id}] ERROR: {exc}', file=sys.stderr)
        try:
            await actuator.safe_stop()
        except Exception:
            pass
    finally:
        try:
            await drone.stop_offboard()
        except Exception:
            pass
        try:
            await drone.land()
        except Exception:
            pass
        await asyncio.sleep(2)
        try:
            await drone.disarm()
        except Exception:
            pass
        logger.save()
        await drone.close()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'[drone {drone_id}] Done')


def cli() -> None:
    parser = argparse.ArgumentParser(description='Follower drone runtime')
    parser.add_argument('--id', type=int, required=True, help='Drone ID (1, 2, or 3)')
    parser.add_argument('--show', action='store_true', help='Show OpenCV debug window')
    args = parser.parse_args()
    asyncio.run(main(args.id, show=args.show))


if __name__ == '__main__':
    cli()
