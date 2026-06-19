#!/usr/bin/env python3
"""
Synchronized startup and sequential alignment example.

The mock observations in this file emulate a future optical CV/LED decoder.
They are not a runtime shortcut between drones.
"""

import asyncio
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from drone_sdk import (  # noqa: E402
    FollowerController,
    FollowerControllerConfig,
    MissionState,
    StartupConfig,
    SwarmStartupCoordinator,
    VisualObservation,
    align_chain_sequentially,
    build_chain_config,
    prepare_swarm_for_start,
    run_follower_controller,
    safe_stop_all,
    wait_for_all_ready_then_start,
)


def ready_observation() -> VisualObservation:
    return VisualObservation(
        target_visible=True,
        horizontal_angle_deg=0.5,
        vertical_angle_deg=0.3,
        target_size=80.0,
        mission_state=MissionState.FOLLOW,
        timestamp=time.monotonic(),
    )


def make_static_provider():
    return ready_observation


async def demo_start_barrier() -> None:
    all_ready_event = asyncio.Event()
    leader_started = []

    async def leader_mission():
        leader_started.append('leader mission started')

    mission_task = asyncio.create_task(
        wait_for_all_ready_then_start(all_ready_event, leader_mission)
    )
    await asyncio.sleep(0)
    print(f'leader before ALL_READY: {bool(leader_started)}')
    all_ready_event.set()
    await mission_task
    print(f'leader after ALL_READY: {bool(leader_started)}')


async def run_real_chain_start(follower_count: int = 3) -> None:
    from drone_sdk import Drone, DroneFollowerActuator

    config = StartupConfig(startup_altitude_m=10.0)
    controller_config = FollowerControllerConfig.responsive()
    links = build_chain_config(follower_count)
    leader = Drone(drone_id=0)
    followers = [Drone(drone_id=link.drone_id) for link in links]
    all_drones = [leader, *followers]
    all_ready_event = asyncio.Event()
    stop_requested = False

    def stop_condition() -> bool:
        return stop_requested

    try:
        await asyncio.gather(*(drone.connect() for drone in all_drones))

        await prepare_swarm_for_start(leader, followers, config)

        observation_providers = [make_static_provider() for _ in followers]
        await align_chain_sequentially(
            followers,
            observation_providers,
            config,
            controller_config,
        )
        all_ready_event.set()

        async def leader_mission():
            await all_ready_event.wait()
            # Put the real leader mission here. It starts only after ALL_READY.
            await asyncio.sleep(0)

        controllers = [
            FollowerController(link.follower_id, link.target_id, controller_config)
            for link in links
        ]
        actuators = [DroneFollowerActuator(drone) for drone in followers]
        follower_tasks = [
            run_follower_controller(
                controller,
                provider,
                actuator,
                stop_condition,
            )
            for controller, provider, actuator in zip(
                controllers,
                observation_providers,
                actuators,
            )
        ]
        await asyncio.gather(leader_mission(), *follower_tasks)
    except KeyboardInterrupt:
        stop_requested = True
    except Exception:
        stop_requested = True
        raise
    finally:
        await safe_stop_all(all_drones)


async def run_with_coordinator(follower_count: int = 3) -> None:
    from drone_sdk import Drone

    config = StartupConfig(startup_altitude_m=10.0)
    coordinator = SwarmStartupCoordinator(config=config)
    links = build_chain_config(follower_count)
    leader = Drone(drone_id=0)
    followers = [Drone(drone_id=link.drone_id) for link in links]
    all_ready_event = asyncio.Event()
    providers = [make_static_provider() for _ in followers]

    try:
        await asyncio.gather(*(drone.connect() for drone in [leader, *followers]))
        await coordinator.prepare_swarm_for_start(
            leader,
            followers,
            providers,
            all_ready_event,
        )
        await all_ready_event.wait()
    finally:
        await safe_stop_all([leader, *followers])


def main() -> None:
    print('Startup flow:')
    print('SPAWN -> CONNECT -> SYNCHRONIZED_TAKEOFF -> WAIT_ALTITUDE -> SETTLE')
    print('-> ALIGN Follower 1 -> ALIGN Follower 2 -> ... -> ALL_READY')
    asyncio.run(demo_start_barrier())


if __name__ == '__main__':
    main()
