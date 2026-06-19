#!/usr/bin/env python3
"""
Final end-to-end CV test (legal Path B): real optical channel, no ground truth.

Single process owns all 4 MAVSDK connections. The leader (drone 0) flies a
defined route AND blinks the two-LED optical protocol. Each follower (1,2,3)
runs the real perception+control chain from its OWN camera:

    camera -> GreenLedDetector -> decoder -> tracker -> adapter
           -> VisualObservation -> FollowerController -> DroneFollowerActuator

Chain: follower 1 tracks leader, 2 tracks 1, 3 tracks 2.

PREREQUISITE: run `bash project_setup.sh` so the 2-lens model is in PX4 (leader
shows exactly led_lens_01 + led_lens_04). Otherwise the detector sees the old
4-green model and cannot decode the blink.

Run via resources/scripts/cv_launch.py (sim + this), or standalone after sim is up.
"""
import asyncio
import math
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
from drone_sdk.two_led_cv import signal_on_for_state
from cv_vision_provider import CVVisionProvider
from results_logger import ResultsLogger

# ─── Configuration ─────────────────────────────────────────────────────────────

FOLLOWER_COUNT = 3
COMMON_ALT_M = 5.0
EKF_SETTLE_S = 15.0
HOVER_SETTLE_S = 8.0
CONTROL_HZ = 10.0
BEACON_HZ = 20.0
WATCHDOG_S = 300.0
FINISH_BROADCAST_S = 8.0   # how long the leader blinks FINISH before landing
ACQUIRE_HOLD_S = 30.0      # leader hovers in place first so followers acquire + form up
REQUIRE_ALL_FOLLOWERS_ACQUIRED = True
STEP_M = 1.0               # leader route step size
STEP_SLEEP_S = 1.2         # sleep per step -> leader cruise ~0.8 m/s (followers max 2 m/s)

# Leader route in its OWN local NED: (north, east, down, yaw_deg, hold_s)
# Simplified: a single straight line North at constant altitude/heading, so the
# whole chain just flies forward — easiest case for acquisition + following.
LEADER_WAYPOINTS = [
    (0.0, 0.0, -COMMON_ALT_M, 0.0, 5),       # settle, face North
    (20.0, 0.0, -COMMON_ALT_M, 0.0, 10),     # straight 20 m North, no turns
]


def _state_str(value):
    return getattr(value, 'value', str(value))


def leader_mask(state: str, t: float) -> str:
    """2-lens mask: index 0 = led_lens_01 (anchor, always on),
    index 1 = led_lens_04 (signal, per protocol timing). Positions 3/4 unused."""
    signal_on = signal_on_for_state(state, t)
    return f"1{'1' if signal_on else '0'}00"


# ─── Leader LED beacon ───────────────────────────────────────────────────────

async def leader_beacon(leader: Drone, beacon_state: list, stop_event: asyncio.Event):
    period = 1.0 / BEACON_HZ
    t = 0.0
    try:
        while not stop_event.is_set():
            leader.set_leds(leader_mask(beacon_state[0], t))
            t += period
            await asyncio.sleep(period)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f'[beacon] error: {e}')


# ─── Leader route ───────────────────────────────────────────────────────────

async def _goto_slow(leader, start, end, yaw, stop_event):
    """Fly from start (n,e,d) to end (n,e,d) in small steps so followers keep up."""
    n0, e0, d0 = start
    n1, e1, d1 = end
    dist = math.sqrt((n1 - n0) ** 2 + (e1 - e0) ** 2 + (d1 - d0) ** 2)
    steps = max(1, int(math.ceil(dist / STEP_M)))
    for k in range(1, steps + 1):
        if stop_event.is_set():
            return
        f = k / steps
        await leader.go_to(n0 + (n1 - n0) * f, e0 + (e1 - e0) * f, d0 + (d1 - d0) * f, yaw_deg=yaw)
        await asyncio.sleep(STEP_SLEEP_S)


async def fly_leader(leader: Drone, beacon_state: list, stop_event: asyncio.Event,
                     acquisition_event: asyncio.Event):
    try:
        # Form-up: hover at the first waypoint blinking FOLLOW so the followers can
        # SEARCH, acquire the LED pair, lock FOLLOW, and close to formation distance
        # BEFORE the leader starts moving (otherwise it flies out of camera range).
        first = LEADER_WAYPOINTS[0]
        await leader.go_to(first[0], first[1], first[2], yaw_deg=first[3])
        print(f'[leader] form-up hover {ACQUIRE_HOLD_S:.0f}s (followers acquire)')
        try:
            await asyncio.wait_for(acquisition_event.wait(), timeout=ACQUIRE_HOLD_S)
            print('[leader] acquisition gate passed — starting route')
        except asyncio.TimeoutError:
            print('[leader] acquisition timeout — chain did not synchronize, aborting route')
            stop_event.set()
            return

        prev = first
        for wp in LEADER_WAYPOINTS[1:]:
            if stop_event.is_set():
                break
            print(f'[leader] -> N={wp[0]} E={wp[1]} D={wp[2]} yaw={wp[3]} (slow)')
            await _goto_slow(leader, prev[:3], wp[:3], wp[3], stop_event)
            await asyncio.sleep(wp[4])
            prev = wp

        # Tell followers to finish via the optical channel, then land.
        print('[leader] route complete — broadcasting FINISH')
        beacon_state[0] = 'FINISH'
        await asyncio.sleep(FINISH_BROADCAST_S)
        await leader.stop_offboard()
        await leader.land()
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f'[leader] route error: {e}')


# ─── Follower task ────────────────────────────────────────────────────────────

def _chain_acquired(follower_status: dict, follower_ids: list[int]) -> bool:
    if REQUIRE_ALL_FOLLOWERS_ACQUIRED:
        return all(
            follower_status.get(fid, {}).get('state') == FollowerState.FOLLOW
            and follower_status.get(fid, {}).get('visible')
            for fid in follower_ids
        )
    return (
        follower_status.get(1, {}).get('state') == FollowerState.FOLLOW
        and follower_status.get(1, {}).get('visible')
    )


async def run_one_follower(controller, provider, actuator, logger, drone_id,
                           stop_event: asyncio.Event, follower_status: dict,
                           follower_ids: list[int], acquisition_event: asyncio.Event):
    period = 1.0 / CONTROL_HZ
    tick = 0
    invisible_ticks = 0
    try:
        while not stop_event.is_set():
            obs = await provider.observe()
            cmd = controller.update(obs)
            await actuator.apply(cmd)
            tick += 1
            if obs.target_visible:
                invisible_ticks = 0
            else:
                invisible_ticks += 1
            follower_status[drone_id] = {
                'state': cmd.state,
                'mission': obs.mission_state,
                'visible': obs.target_visible,
            }
            if not acquisition_event.is_set() and _chain_acquired(follower_status, follower_ids):
                acquired = ', '.join(
                    f'{fid}:{_state_str(follower_status[fid]["state"])}'
                    for fid in follower_ids
                )
                print(f'[main] acquisition gate satisfied ({acquired})', flush=True)
                acquisition_event.set()
            if tick % 20 == 0:  # ~ every 2s, into run.log
                print(f'[follower {drone_id}] state={_state_str(cmd.state)} '
                      f'mission={_state_str(obs.mission_state)} vis={obs.target_visible} '
                      f'h={obs.horizontal_angle_deg:.1f} v={obs.vertical_angle_deg:.1f} '
                      f'size={obs.target_size:.0f} fwd={cmd.forward_m_s:.2f} '
                      f'yaw_rate={cmd.yaw_rate_deg_s:.1f}', flush=True)
            if invisible_ticks == int(CONTROL_HZ * 5):
                print(f'[follower {drone_id}] WARN: no target LEDs visible for 5s '
                      f'(mission={_state_str(obs.mission_state)}, state={_state_str(cmd.state)})',
                      flush=True)
            logger.log(drone_id, {
                'follower': controller.follower_id,
                'target': controller.target_id,
                'state': _state_str(cmd.state),
                'mission': _state_str(obs.mission_state),
                'visible': obs.target_visible,
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
    beacon_state = ['FOLLOW']

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, RuntimeError):
            signal.signal(sig, lambda *_: stop_event.set())

    print('[main] connecting all drones...')
    await asyncio.gather(*(d.connect() for d in drones.values()))
    print('[main] all connected')

    # Start the LED beacon IMMEDIATELY on EVERY drone (leader + followers) before
    # takeoff, so all of them show the green anchor + red signal already on the
    # ground. Lets you visually confirm the two colours on the whole swarm before
    # the route begins, without sending manual gz topic commands.
    beacon_tasks = [
        asyncio.create_task(leader_beacon(d, beacon_state, stop_event))
        for d in drones.values()
    ]
    print('[main] LED beacon started on all drones (green anchor + red signal)')

    # Start follower cameras FIRST, open their windows, and wait for real frames
    # (with a live preview) before taking off — so nobody flies blind and you can
    # see the camera feed immediately instead of after takeoff.
    follower_ids = list(range(1, FOLLOWER_COUNT + 1))
    for fid in follower_ids:
        drones[fid].start_camera()
    providers = {fid: CVVisionProvider(drones[fid], show=True, window_name=f'Follower {fid}')
                 for fid in follower_ids}
    print('[main] cameras started — waiting for frames (live preview)...')
    for _ in range(150):  # up to ~30s
        for fid in follower_ids:
            await providers[fid].observe()  # spins, draws the window
        ready = [fid for fid in follower_ids if drones[fid].camera_frame() is not None]
        if len(ready) == len(follower_ids):
            print('[main] all follower cameras streaming')
            break
        await asyncio.sleep(0.2)
    else:
        print(f'[main] WARN: cameras streaming only for {ready} — proceeding anyway')

    print(f'[main] waiting {EKF_SETTLE_S}s for EKF to stabilize...')
    await asyncio.sleep(EKF_SETTLE_S)

    try:
        async def arm_takeoff(i, d):
            await d.arm()
            await d.set_takeoff_altitude(COMMON_ALT_M + i * 1.0)
            await d.takeoff()

        print('[main] arming and taking off (staggered altitudes)...')
        await asyncio.gather(*(arm_takeoff(i, d) for i, d in drones.items()))
        await asyncio.sleep(HOVER_SETTLE_S)

        print('[main] starting offboard on all drones...')
        await asyncio.gather(*(d.start_offboard() for d in drones.values()))

        # Followers: cameras already started above; build the perception+control chain.
        links = build_chain_config(FOLLOWER_COUNT)
        follower_status = {}
        acquisition_event = asyncio.Event()
        follower_tasks = []
        for link in links:
            fid = link.drone_id
            controller = FollowerController(
                link.follower_id, link.target_id,
                FollowerControllerConfig(control_rate_hz=CONTROL_HZ, search_yaw_rate=20.0),
            )
            actuator = DroneFollowerActuator(drones[fid])
            provider = providers[fid]  # reuse the preview provider (windows already open)
            print(f'[main] {link.follower_id} (drone {fid}) -> {link.target_id}')
            follower_tasks.append(asyncio.create_task(
                run_one_follower(
                    controller, provider, actuator, logger, fid, stop_event,
                    follower_status, follower_ids, acquisition_event,
                )
            ))

        leader_task = asyncio.create_task(
            fly_leader(drones[0], beacon_state, stop_event, acquisition_event)
        )
        wd_task = asyncio.create_task(watchdog(stop_event))

        await asyncio.gather(leader_task, *follower_tasks)
        for bt in beacon_tasks:
            bt.cancel()
        wd_task.cancel()

    finally:
        print('[main] shutting down — landing all drones')
        for d in drones.values():
            try:
                d.led_off()
            except Exception:
                pass
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
