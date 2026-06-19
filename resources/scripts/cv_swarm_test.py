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
    MissionState,
    VisualObservation,
    build_chain_config,
)
from drone_sdk.two_led_cv import signal_on_for_state
from cv_vision_provider import CVVisionProvider
from results_logger import ResultsLogger

# ─── Configuration ─────────────────────────────────────────────────────────────

FOLLOWER_COUNT = 3
COMMON_ALT_M = 10.0
EKF_SETTLE_S = 15.0
HOVER_SETTLE_S = 8.0
TAKEOFF_TIMEOUT_S = 35.0
TAKEOFF_TOLERANCE_M = 0.75
CONTROL_HZ = 10.0
BEACON_HZ = 20.0
TRUTH_LOG_HZ = 1.0
WATCHDOG_S = 300.0
FINISH_BROADCAST_S = 15.0  # how long the leader blinks FINISH before landing
ACQUIRE_HOLD_S = 30.0      # leader hovers in place first so followers acquire + form up
REQUIRE_ALL_FOLLOWERS_ACQUIRED = True
ROUTE_RECOVERY_PAUSE_S = 20.0
ROUTE_RECOVERY_POLL_S = 0.5
STEP_M = 1.0               # leader route step size
STEP_SLEEP_S = 2.5         # sleep per step -> leader cruise ~0.4 m/s while CV is noisy
TRAIN_YAW_RAD = 3.7346
TRAIN_YAW_DEG = math.degrees(TRAIN_YAW_RAD)
STRAIGHT_ROUTE_M = 20.0
STRAIGHT_ROUTE_N = STRAIGHT_ROUTE_M * math.cos(TRAIN_YAW_RAD)
STRAIGHT_ROUTE_E = STRAIGHT_ROUTE_M * math.sin(TRAIN_YAW_RAD)
SPAWN_Z_M = 1.4
TRAIN_SPACING_M = 4.0
ACQUIRE_MIN_TARGET_SIZE = 45.0
ACQUIRE_MAX_TARGET_SIZE = 180.0
SPAWN_LEADER_E = 127.0
SPAWN_LEADER_N = 52.67
SPAWNS = {
    0: (SPAWN_LEADER_N, SPAWN_LEADER_E, SPAWN_Z_M),
}
for _idx in range(1, FOLLOWER_COUNT + 1):
    SPAWNS[_idx] = (
        SPAWNS[0][0] - _idx * TRAIN_SPACING_M * math.sin(TRAIN_YAW_RAD),
        SPAWNS[0][1] - _idx * TRAIN_SPACING_M * math.cos(TRAIN_YAW_RAD),
        SPAWN_Z_M,
    )

# Leader route in its OWN local NED: (north, east, down, yaw_deg, hold_s)
# Single straight segment along the same yaw used by the spawn train. This keeps
# the leader from turning at route start and keeps follower cameras aimed forward.
LEADER_WAYPOINTS = [
    (0.0, 0.0, -COMMON_ALT_M, TRAIN_YAW_DEG, 5),
    (STRAIGHT_ROUTE_N, STRAIGHT_ROUTE_E, -COMMON_ALT_M, TRAIN_YAW_DEG, 10),
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


async def _wait_until_altitude(drone: Drone, target_alt_m: float, timeout_s: float) -> float:
    deadline = asyncio.get_running_loop().time() + timeout_s
    last_alt = 0.0
    while asyncio.get_running_loop().time() < deadline:
        pos = await drone.position_ned()
        last_alt = -pos.down_m
        if abs(last_alt - target_alt_m) <= TAKEOFF_TOLERANCE_M:
            return last_alt
        await asyncio.sleep(0.25)
    raise RuntimeError(
        f'drone {drone.drone_id} takeoff altitude timeout: '
        f'target={target_alt_m:.1f}m last={last_alt:.1f}m'
    )


def _force_search_when_invisible(obs: VisualObservation) -> VisualObservation:
    if obs.target_visible:
        return obs
    mission = _state_str(obs.mission_state).upper()
    if mission in ('FOLLOW', 'FINISH'):
        return obs
    return VisualObservation(
        target_visible=False,
        horizontal_angle_deg=obs.horizontal_angle_deg,
        vertical_angle_deg=obs.vertical_angle_deg,
        target_size=obs.target_size,
        mission_state=MissionState.FOLLOW,
        timestamp=obs.timestamp,
    )


def _round_or_none(value, digits: int):
    if isinstance(value, (int, float)) and math.isfinite(value):
        return round(value, digits)
    return None


def _fmt(value, digits: int = 1) -> str:
    if isinstance(value, (int, float)) and math.isfinite(value):
        return f'{value:.{digits}f}'
    return 'None'


async def _altitude_or_none(drone: Drone):
    try:
        pos = await drone.position_ned()
        return -pos.down_m
    except Exception:
        return None


async def _truth_snapshot(drones: dict[int, Drone]) -> dict[int, dict]:
    snapshot = {}
    for drone_id, drone in drones.items():
        pos = await drone.position_ned()
        heading = await drone.heading()
        spawn_n, spawn_e, spawn_z = SPAWNS[drone_id]
        world_n = spawn_n + pos.north_m
        world_e = spawn_e + pos.east_m
        altitude = -pos.down_m
        snapshot[drone_id] = {
            'world_n': world_n,
            'world_e': world_e,
            'altitude_m': altitude,
            'heading_deg': heading,
            'local_n': pos.north_m,
            'local_e': pos.east_m,
            'local_down': pos.down_m,
            'spawn_z': spawn_z,
        }
    return snapshot


def _target_drone_id(drone_id: int) -> int:
    return 0 if drone_id == 1 else drone_id - 1


def _relative_truth(follower: dict, target: dict) -> dict:
    dn = target['world_n'] - follower['world_n']
    de = target['world_e'] - follower['world_e']
    dz = target['altitude_m'] - follower['altitude_m']
    horizontal_m = math.hypot(dn, de)
    distance_m = math.sqrt(horizontal_m ** 2 + dz ** 2)
    bearing_deg = math.degrees(math.atan2(de, dn))
    heading_error_deg = ((bearing_deg - follower['heading_deg'] + 180.0) % 360.0) - 180.0
    return {
        'truth_dn_m': dn,
        'truth_de_m': de,
        'truth_dalt_m': dz,
        'truth_horizontal_m': horizontal_m,
        'truth_distance_m': distance_m,
        'truth_bearing_deg': bearing_deg,
        'truth_heading_error_deg': heading_error_deg,
    }


# ─── Leader route ───────────────────────────────────────────────────────────

def _route_chain_ready(follower_status: dict, follower_ids: list[int]) -> bool:
    return all(
        follower_status.get(fid, {}).get('state') == FollowerState.FOLLOW
        and follower_status.get(fid, {}).get('visible')
        for fid in follower_ids
    )


async def _wait_route_chain_ready(leader, current, yaw, stop_event, follower_status, follower_ids):
    if _route_chain_ready(follower_status, follower_ids):
        return
    missing = ', '.join(
        f'{fid}:{_state_str(follower_status.get(fid, {}).get("state", "?"))}'
        f'/vis={follower_status.get(fid, {}).get("visible", "?")}'
        for fid in follower_ids
    )
    print(f'[leader] pausing route for follower recovery ({missing})', flush=True)
    deadline = asyncio.get_running_loop().time() + ROUTE_RECOVERY_PAUSE_S
    while not stop_event.is_set() and asyncio.get_running_loop().time() < deadline:
        await leader.go_to(current[0], current[1], current[2], yaw_deg=yaw)
        await asyncio.sleep(ROUTE_RECOVERY_POLL_S)
        if _route_chain_ready(follower_status, follower_ids):
            print('[leader] follower chain recovered — resuming route', flush=True)
            return
    print('[leader] follower chain still degraded — continuing slowly', flush=True)


async def _goto_slow(leader, start, end, yaw, stop_event, follower_status, follower_ids):
    """Fly from start (n,e,d) to end (n,e,d) in small steps so followers keep up."""
    n0, e0, d0 = start
    n1, e1, d1 = end
    dist = math.sqrt((n1 - n0) ** 2 + (e1 - e0) ** 2 + (d1 - d0) ** 2)
    steps = max(1, int(math.ceil(dist / STEP_M)))
    current = start
    for k in range(1, steps + 1):
        if stop_event.is_set():
            return
        await _wait_route_chain_ready(leader, current, yaw, stop_event, follower_status, follower_ids)
        f = k / steps
        current = (n0 + (n1 - n0) * f, e0 + (e1 - e0) * f, d0 + (d1 - d0) * f)
        await leader.go_to(current[0], current[1], current[2], yaw_deg=yaw)
        await asyncio.sleep(STEP_SLEEP_S)


async def fly_leader(leader: Drone, beacon_state: list, stop_event: asyncio.Event,
                     acquisition_event: asyncio.Event, follower_status: dict,
                     follower_ids: list[int]):
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
            await _goto_slow(
                leader, prev[:3], wp[:3], wp[3], stop_event,
                follower_status, follower_ids,
            )
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
            and _target_size_ready(follower_status.get(fid, {}).get('size'))
            for fid in follower_ids
        )
    return (
        follower_status.get(1, {}).get('state') == FollowerState.FOLLOW
        and follower_status.get(1, {}).get('visible')
        and _target_size_ready(follower_status.get(1, {}).get('size'))
    )


def _target_size_ready(size) -> bool:
    return (
        isinstance(size, (int, float))
        and math.isfinite(size)
        and ACQUIRE_MIN_TARGET_SIZE <= size <= ACQUIRE_MAX_TARGET_SIZE
    )


async def run_one_follower(controller, provider, actuator, logger, drone_id,
                           stop_event: asyncio.Event, follower_status: dict,
                           follower_ids: list[int], acquisition_event: asyncio.Event,
                           beacon_state: list):
    period = 1.0 / CONTROL_HZ
    tick = 0
    invisible_ticks = 0
    last_reported_state = None
    last_reported_visible = None
    last_reported_mission = None
    try:
        while not stop_event.is_set():
            raw_obs = await provider.observe()
            vision_debug = getattr(provider, 'last_debug', {})
            obs = _force_search_when_invisible(raw_obs)
            cmd = controller.update(obs)
            beacon_state[0] = _state_str(cmd.relay_state)
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
                'size': obs.target_size,
            }
            current_state = _state_str(cmd.state)
            current_visible = obs.target_visible
            current_mission = _state_str(obs.mission_state)
            if (
                current_state != last_reported_state
                or current_visible != last_reported_visible
                or current_mission != last_reported_mission
            ):
                print(
                    f'[follower {drone_id}] transition '
                    f'state={current_state} mission={current_mission} '
                    f'raw={_state_str(raw_obs.mission_state)} relay={_state_str(cmd.relay_state)} '
                    f'vis={current_visible} anchor={vision_debug.get("anchor_visible")} '
                    f'signal={vision_debug.get("signal_visible")} '
                    f'sig_ratio={_fmt(vision_debug.get("signal_on_ratio"), 2)} '
                    f'trans_s={_fmt(vision_debug.get("transitions_per_s"), 2)} '
                    f'range={_fmt(vision_debug.get("estimated_range_m"))} '
                    f'h={obs.horizontal_angle_deg:.1f} v={obs.vertical_angle_deg:.1f} '
                    f'size={obs.target_size:.0f}',
                    flush=True,
                )
                last_reported_state = current_state
                last_reported_visible = current_visible
                last_reported_mission = current_mission
            if not acquisition_event.is_set() and _chain_acquired(follower_status, follower_ids):
                acquired = ', '.join(
                    f'{fid}:{_state_str(follower_status[fid]["state"])}'
                    f'/size={follower_status[fid].get("size", 0):.0f}'
                    for fid in follower_ids
                )
                print(f'[main] acquisition gate satisfied ({acquired})', flush=True)
                acquisition_event.set()
            if tick % 20 == 0:  # ~ every 2s, into run.log
                altitude_m = await _altitude_or_none(actuator.drone)
                print(f'[follower {drone_id}] state={_state_str(cmd.state)} '
                      f'mission={_state_str(obs.mission_state)} raw={_state_str(raw_obs.mission_state)} '
                      f'relay={_state_str(cmd.relay_state)} vis={obs.target_visible} '
                      f'anchor={vision_debug.get("anchor_visible")} '
                      f'signal={vision_debug.get("signal_visible")} '
                      f'sig_ratio={_fmt(vision_debug.get("signal_on_ratio"), 2)} '
                      f'trans_s={_fmt(vision_debug.get("transitions_per_s"), 2)} '
                      f'range={_fmt(vision_debug.get("estimated_range_m"))} '
                      f'alt={_fmt(altitude_m)} '
                      f'h={obs.horizontal_angle_deg:.1f} v={obs.vertical_angle_deg:.1f} '
                      f'size={obs.target_size:.0f} fwd={cmd.forward_m_s:.2f} '
                      f'down={cmd.down_m_s:.2f} yaw_rate={cmd.yaw_rate_deg_s:.1f}',
                      flush=True)
            if invisible_ticks == int(CONTROL_HZ * 5):
                print(f'[follower {drone_id}] WARN: no target LEDs visible for 5s '
                      f'(mission={_state_str(obs.mission_state)}, state={_state_str(cmd.state)})',
                      flush=True)
            logger.log(drone_id, {
                'follower': controller.follower_id,
                'target': controller.target_id,
                'state': _state_str(cmd.state),
                'mission': _state_str(obs.mission_state),
                'raw_mission': _state_str(raw_obs.mission_state),
                'relay': _state_str(cmd.relay_state),
                'visible': obs.target_visible,
                'raw_visible': raw_obs.target_visible,
                'anchor_visible': vision_debug.get('anchor_visible'),
                'signal_visible': vision_debug.get('signal_visible'),
                'decoder_state': vision_debug.get('decoder_state'),
                'decoder_anchor_ratio': _round_or_none(vision_debug.get('anchor_ratio'), 3),
                'decoder_signal_ratio': _round_or_none(vision_debug.get('signal_on_ratio'), 3),
                'decoder_transitions_s': _round_or_none(vision_debug.get('transitions_per_s'), 3),
                'range_m': _round_or_none(vision_debug.get('estimated_range_m'), 3),
                'led_distance_px': _round_or_none(vision_debug.get('led_distance_px'), 2),
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


async def truth_logger(drones: dict[int, Drone], logger: ResultsLogger,
                       follower_status: dict, stop_event: asyncio.Event):
    period = 1.0 / TRUTH_LOG_HZ
    try:
        while not stop_event.is_set():
            try:
                snapshot = await _truth_snapshot(drones)
                link_parts = []
                for fid in range(1, FOLLOWER_COUNT + 1):
                    tid = _target_drone_id(fid)
                    rel = _relative_truth(snapshot[fid], snapshot[tid])
                    status = follower_status.get(fid, {})
                    link_parts.append(
                        f'{fid}->{tid}:d={rel["truth_distance_m"]:.1f}m '
                        f'h={rel["truth_horizontal_m"]:.1f}m '
                        f'dalt={rel["truth_dalt_m"]:.1f}m '
                        f'bear_err={rel["truth_heading_error_deg"]:.1f}deg '
                        f'state={_state_str(status.get("state", "?"))} '
                        f'vis={status.get("visible", "?")}'
                    )
                    logger.log(fid, {
                        'kind': 'truth',
                        'target_drone_id': tid,
                        'world_n': _round_or_none(snapshot[fid]['world_n'], 3),
                        'world_e': _round_or_none(snapshot[fid]['world_e'], 3),
                        'altitude_m': _round_or_none(snapshot[fid]['altitude_m'], 3),
                        'heading_deg': _round_or_none(snapshot[fid]['heading_deg'], 3),
                        'target_world_n': _round_or_none(snapshot[tid]['world_n'], 3),
                        'target_world_e': _round_or_none(snapshot[tid]['world_e'], 3),
                        'target_altitude_m': _round_or_none(snapshot[tid]['altitude_m'], 3),
                        'target_heading_deg': _round_or_none(snapshot[tid]['heading_deg'], 3),
                        **{
                            key: _round_or_none(value, 3)
                            for key, value in rel.items()
                        },
                        'controller_state': _state_str(status.get('state', 'UNKNOWN')),
                        'controller_mission': _state_str(status.get('mission', 'UNKNOWN')),
                        'controller_visible': status.get('visible'),
                    })
                print('[truth] ' + ' | '.join(link_parts), flush=True)
            except Exception as e:
                print(f'[truth] error: {e}', flush=True)
            await asyncio.sleep(period)
    except asyncio.CancelledError:
        pass


# ─── Orchestration ───────────────────────────────────────────────────────────

async def main():
    rclpy.init()
    drones = {i: Drone(drone_id=i) for i in range(FOLLOWER_COUNT + 1)}
    logger = ResultsLogger()
    stop_event = asyncio.Event()
    beacon_states = {0: ['FOLLOW']}
    for fid in range(1, FOLLOWER_COUNT + 1):
        beacon_states[fid] = ['HOLD']

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, RuntimeError):
            signal.signal(sig, lambda *_: stop_event.set())

    print('[main] connecting all drones...')
    await asyncio.gather(*(d.connect() for d in drones.values()))
    print('[main] all connected')

    # Start the LED beacon IMMEDIATELY on EVERY drone before takeoff. The leader
    # advertises FOLLOW; followers relay their own controller state so the next
    # drone waits for the previous link to become valid.
    beacon_tasks = [
        asyncio.create_task(leader_beacon(d, beacon_states[i], stop_event))
        for i, d in drones.items()
    ]
    print('[main] LED beacon started on all drones (leader FOLLOW, followers HOLD until acquired)')

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
            await d.set_takeoff_altitude(COMMON_ALT_M)
            await d.takeoff()

        print(f'[main] arming and taking off to {COMMON_ALT_M:.1f}m...')
        await asyncio.gather(*(arm_takeoff(i, d) for i, d in drones.items()))
        print('[main] waiting for verified takeoff altitude...')
        reached = await asyncio.gather(*(
            _wait_until_altitude(d, COMMON_ALT_M, TAKEOFF_TIMEOUT_S)
            for d in drones.values()
        ))
        print('[main] takeoff altitude verified: ' + ', '.join(
            f'drone {i}={alt:.1f}m' for i, alt in zip(drones.keys(), reached)
        ))
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
                FollowerControllerConfig(
                    control_rate_hz=CONTROL_HZ,
                    search_yaw_rate=20.0,
                    kp_forward=0.05,
                    max_forward_speed=1.5,
                    lost_timeout=1.0,
                    lost_command_memory_s=0.8,
                ),
            )
            actuator = DroneFollowerActuator(drones[fid])
            provider = providers[fid]  # reuse the preview provider (windows already open)
            print(f'[main] {link.follower_id} (drone {fid}) -> {link.target_id}')
            follower_tasks.append(asyncio.create_task(
                run_one_follower(
                    controller, provider, actuator, logger, fid, stop_event,
                    follower_status, follower_ids, acquisition_event,
                    beacon_states[fid],
                )
            ))

        truth_task = asyncio.create_task(
            truth_logger(drones, logger, follower_status, stop_event)
        )
        leader_task = asyncio.create_task(
            fly_leader(
                drones[0], beacon_states[0], stop_event, acquisition_event,
                follower_status, follower_ids,
            )
        )
        wd_task = asyncio.create_task(watchdog(stop_event))

        await asyncio.gather(leader_task, *follower_tasks)
        for bt in beacon_tasks:
            bt.cancel()
        truth_task.cancel()
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
