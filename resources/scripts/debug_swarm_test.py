#!/usr/bin/env python3
"""
DEBUG / EVAL ONLY — ground-truth cheat, NEVER SUBMIT.

Single-process end-to-end test of the follower controller in PX4 SITL + Gazebo.

One process owns ALL MAVSDK connections (drones 0..3, ports 14540..14543 — no
port clash). It flies the leader (drone 0) along a defined route and runs the
chain of follower controllers (drone 1 -> leader, 2 -> 1, 3 -> 2), feeding each
controller a ground-truth VisualObservation from DebugVisionProvider instead of
a camera. This validates the controller + actuator geometry before the real CV
pipeline exists.

Why one process: two MAVSDK clients cannot bind the same UDP port, so a
per-process follower could not open a second connection to read its target.
Owning every connection here sidesteps that and gives us every drone's position
in-process.

Run via resources/scripts/debug_launch.py (sim + this script), or standalone in
a second terminal after the sim is up:  python3 debug_swarm_test.py
"""
import asyncio
import os
import signal
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.dirname(__file__))

import rclpy

from drone_sdk import Drone
from drone_sdk.follower_controller import (
    DroneFollowerActuator,
    FollowerController,
    FollowerControllerConfig,
    FollowerState,
    build_chain_config,
)
from debug_vision_provider import DebugVisionProvider
from results_logger import ResultsLogger

# ─── Configuration ─────────────────────────────────────────────────────────────

FOLLOWER_COUNT = 3            # drones 0 (leader) + 1,2,3 (followers)
COMMON_ALT_M = 5.0           # leader cruise altitude
EKF_SETTLE_S = 15.0          # wait after connect before arming (matches follower.py)
HOVER_SETTLE_S = 8.0         # wait for takeoff to reach hover
CONTROL_HZ = 10.0
WATCHDOG_S = 180.0           # hard cap on the whole test
STRAIGHT_DISTANCE_M = 10.0
FINAL_HOVER_S = 5.0

# Gazebo world spawn poses (x=East, y=North, z=Up) — must match solution_launch.py
SPAWNS = {
    0: (127.0, 52.67, 1.4),
    1: (129.92, 52.852, 1.4),
    2: (129.08, 54.095, 1.4),
    3: (128.24, 55.339, 1.4),
}

# Leader route in its OWN local NED: (north, east, down, yaw_deg, hold_s).
# Straight mission only: take off, fly forward, hold briefly, then land.
LEADER_WAYPOINTS = [
    (0.0, 0.0, -COMMON_ALT_M, 0.0, 5),
    (STRAIGHT_DISTANCE_M, 0.0, -COMMON_ALT_M, 0.0, FINAL_HOVER_S),
]


def _state_str(value):
    return getattr(value, 'value', str(value))


# ─── Leader route ───────────────────────────────────────────────────────────────

async def fly_leader(leader: Drone, stop_event: asyncio.Event):
    try:
        for north, east, down, yaw, hold in LEADER_WAYPOINTS:
            if stop_event.is_set():
                break
            print(f'[leader] -> N={north} E={east} D={down} yaw={yaw}')
            await leader.go_to(north, east, down, yaw_deg=yaw)
            await asyncio.sleep(hold)
        print('[leader] route complete — landing')
        await leader.stop_offboard()
        await leader.land()
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f'[leader] route error: {e}')


# ─── Follower task ────────────────────────────────────────────────────────────

async def run_one_follower(controller, provider, actuator, logger, drone_id,
                           stop_event: asyncio.Event):
    period = 1.0 / CONTROL_HZ
    try:
        while not stop_event.is_set():
            obs = await provider.observe()
            cmd = controller.update(obs)
            await actuator.apply(cmd)
            logger.log(drone_id, {
                'follower': controller.follower_id,
                'target': controller.target_id,
                'state': _state_str(cmd.state),
                'mission': _state_str(obs.mission_state),
                'h_angle': round(obs.horizontal_angle_deg, 3),
                'v_angle': round(obs.vertical_angle_deg, 3),
                'size': round(obs.target_size, 2),
                'fwd': round(cmd.forward_m_s, 3),
                'down': round(cmd.down_m_s, 3),
                'yaw_rate': round(cmd.yaw_rate_deg_s, 3),
            })
            if cmd.state == FollowerState.FINISH:
                print(f'[follower {drone_id}] FINISH')
                break
            await asyncio.sleep(period)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f'[follower {drone_id}] error: {e}')
    finally:
        try:
            await actuator.safe_stop()
        except Exception:
            pass


async def watchdog(stop_event: asyncio.Event):
    try:
        await asyncio.sleep(WATCHDOG_S)
        print('[watchdog] time cap reached — stopping')
        stop_event.set()
    except asyncio.CancelledError:
        pass


# ─── Orchestration ───────────────────────────────────────────────────────────

async def main():
    rclpy.init()
    drones = {i: Drone(drone_id=i) for i in range(FOLLOWER_COUNT + 1)}
    logger = ResultsLogger()
    stop_event = asyncio.Event()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, RuntimeError):
            signal.signal(sig, lambda *_: stop_event.set())

    print('[main] connecting all drones...')
    await asyncio.gather(*(d.connect() for d in drones.values()))
    print('[main] all connected')

    # Sanity log: each drone's local NED should start near (0,0,~0)
    for i, d in drones.items():
        pos = await d.position_ned()
        print(f'[main] drone {i} initial NED: '
              f'N={pos.north_m:.2f} E={pos.east_m:.2f} D={pos.down_m:.2f}')

    print(f'[main] waiting {EKF_SETTLE_S}s for EKF to stabilize...')
    await asyncio.sleep(EKF_SETTLE_S)

    try:
        # Arm + takeoff with altitude stagger to avoid collisions on climb.
        async def arm_takeoff(i, d):
            await d.arm()
            await d.set_takeoff_altitude(COMMON_ALT_M + i * 1.0)
            await d.takeoff()

        print('[main] arming and taking off (staggered altitudes)...')
        await asyncio.gather(*(arm_takeoff(i, d) for i, d in drones.items()))
        await asyncio.sleep(HOVER_SETTLE_S)

        print('[main] starting offboard on all drones...')
        await asyncio.gather(*(d.start_offboard() for d in drones.values()))

        # Build the follower chain.
        links = build_chain_config(FOLLOWER_COUNT)
        follower_tasks = []
        for link in links:
            fid = link.drone_id
            tid = 0 if link.target_id == 'leader' else int(link.target_id.split('_')[1])
            controller = FollowerController(
                link.follower_id, link.target_id,
                FollowerControllerConfig(control_rate_hz=CONTROL_HZ),
            )
            actuator = DroneFollowerActuator(drones[fid])
            provider = DebugVisionProvider(
                follower_drone=drones[fid],
                target_drone=drones[tid],
                leader_drone=drones[0],
                follower_spawn=SPAWNS[fid],
                target_spawn=SPAWNS[tid],
                leader_spawn=SPAWNS[0],
            )
            print(f'[main] {link.follower_id} (drone {fid}) -> {link.target_id} (drone {tid})')
            follower_tasks.append(asyncio.create_task(
                run_one_follower(controller, provider, actuator, logger, fid, stop_event)
            ))

        leader_task = asyncio.create_task(fly_leader(drones[0], stop_event))
        wd_task = asyncio.create_task(watchdog(stop_event))

        await asyncio.gather(leader_task, *follower_tasks)
        wd_task.cancel()

    finally:
        print('[main] shutting down — landing all drones')
        for d in drones.values():
            try:
                await d.stop_offboard()
            except Exception:
                pass
        for d in drones.values():
            try:
                await d.land()
            except Exception:
                pass
        await asyncio.sleep(10)
        for d in drones.values():
            try:
                await d.disarm()
            except Exception:
                pass
        logger.save()
        for d in drones.values():
            try:
                await d.close()
            except Exception:
                pass
        rclpy.shutdown()
        print('[main] done')


if __name__ == '__main__':
    asyncio.run(main())
