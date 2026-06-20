from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Awaitable, Callable, Iterable, Optional, Sequence

from .follower_controller import (
    DroneFollowerActuator,
    FollowerCommand,
    FollowerControllerConfig,
    FollowerState,
    MissionState,
    VisualObservation,
    build_chain_config,
    normalize_mission_state,
)


class StartupError(RuntimeError):
    pass


class StartupState(str, Enum):
    IDLE = 'IDLE'
    CONNECTING = 'CONNECTING'
    TAKING_OFF = 'TAKING_OFF'
    WAITING_ALTITUDE = 'WAITING_ALTITUDE'
    STABILIZING = 'STABILIZING'
    ALIGNING = 'ALIGNING'
    READY = 'READY'
    FAILED = 'FAILED'


class AlignmentStatus(str, Enum):
    ALIGNING = 'ALIGNING'
    READY = 'READY'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class StartupConfig:
    startup_altitude_m: float = 10.0
    startup_timeout_s: float = 30.0
    startup_settle_time_s: float = 2.0
    startup_height_tolerance_m: float = 0.75
    startup_poll_interval_s: float = 0.25
    alignment_timeout_s: float = 20.0
    alignment_required_frames: int = 3
    alignment_horizontal_tolerance_deg: float = 2.0
    alignment_vertical_tolerance_deg: float = 2.0
    alignment_size_tolerance: float = 5.0
    alignment_max_forward_speed: float = 0.4
    alignment_max_yaw_rate: float = 8.0
    alignment_max_vertical_speed: float = 0.3
    alignment_observation_timeout_s: float = 0.5
    alignment_search_yaw_rate: float = 5.0


@dataclass(frozen=True)
class DroneStartupStatus:
    drone_id: str
    state: StartupState
    ready: bool = False
    altitude_m: Optional[float] = None
    error: Optional[str] = None


@dataclass(frozen=True)
class AlignmentResult:
    follower_id: str
    target_id: str
    status: AlignmentStatus
    ready_frames: int = 0
    last_command: Optional[FollowerCommand] = None
    error: Optional[str] = None


@dataclass
class SwarmStartupCoordinator:
    config: StartupConfig = field(default_factory=StartupConfig)
    controller_config: FollowerControllerConfig = field(
        default_factory=FollowerControllerConfig.stable
    )
    state: StartupState = StartupState.IDLE
    statuses: list[DroneStartupStatus] = field(default_factory=list)
    alignment_results: list[AlignmentResult] = field(default_factory=list)

    async def prepare_swarm_for_start(
        self,
        leader,
        followers: Sequence,
        observation_providers: Optional[Sequence[Callable[[], ObservationResult]]] = None,
        all_ready_event: Optional[asyncio.Event] = None,
    ) -> list[AlignmentResult]:
        self.state = StartupState.CONNECTING
        await prepare_swarm_for_start(leader, followers, self.config)

        if observation_providers is None:
            self.state = StartupState.READY
            if all_ready_event is not None:
                all_ready_event.set()
            return []

        self.state = StartupState.ALIGNING
        try:
            self.alignment_results = await align_chain_sequentially(
                followers,
                observation_providers,
                self.config,
                self.controller_config,
            )
        except Exception:
            self.state = StartupState.FAILED
            await safe_stop_all([leader, *followers])
            raise

        self.state = StartupState.READY
        if all_ready_event is not None:
            all_ready_event.set()
        return self.alignment_results


ObservationResult = Optional[VisualObservation] | Awaitable[Optional[VisualObservation]]


async def safe_stop_all(drones: Iterable) -> None:
    for drone in drones:
        try:
            heading = await drone.heading()
            await drone.set_velocity(0.0, 0.0, 0.0, yaw_deg=heading)
        except Exception:
            pass


async def prepare_swarm_for_start(
    leader,
    followers: Sequence,
    config: Optional[StartupConfig] = None,
) -> list[DroneStartupStatus]:
    startup_config = config or StartupConfig()
    drones = [leader, *followers]
    statuses: list[DroneStartupStatus] = []

    try:
        for drone in drones:
            if hasattr(drone, 'connected') and not drone.connected:
                raise StartupError(f'drone {drone_id(drone)} is not connected')

        for drone in drones:
            await drone.arm()

        await asyncio.gather(*(
            drone.takeoff(altitude_m=startup_config.startup_altitude_m)
            for drone in drones
        ))

        await asyncio.gather(*(
            _wait_until_startup_altitude(drone, startup_config)
            for drone in drones
        ))

        if startup_config.startup_settle_time_s > 0.0:
            await asyncio.sleep(startup_config.startup_settle_time_s)

        for drone in drones:
            await drone.start_offboard()
        await safe_stop_all(drones)

        for drone in drones:
            statuses.append(
                DroneStartupStatus(
                    drone_id=drone_id(drone),
                    state=StartupState.READY,
                    ready=True,
                    altitude_m=await _own_altitude_or_none(drone),
                )
            )
        return statuses
    except Exception as exc:
        await safe_stop_all(drones)
        message = str(exc)
        statuses.clear()
        for drone in drones:
            statuses.append(
                DroneStartupStatus(
                    drone_id=drone_id(drone),
                    state=StartupState.FAILED,
                    ready=False,
                    error=message,
                )
            )
        if isinstance(exc, StartupError):
            raise
        raise StartupError(f'swarm startup failed: {exc}') from exc


async def _wait_until_startup_altitude(drone, config: StartupConfig) -> None:
    deadline = time.monotonic() + config.startup_timeout_s
    last_altitude: Optional[float] = None

    while time.monotonic() < deadline:
        altitude = await _own_altitude_or_none(drone)
        if altitude is None:
            await asyncio.sleep(config.startup_settle_time_s)
            return
        last_altitude = altitude
        if abs(altitude - config.startup_altitude_m) <= config.startup_height_tolerance_m:
            return
        await asyncio.sleep(config.startup_poll_interval_s)

    raise StartupError(
        'startup altitude timeout for '
        f'{drone_id(drone)}: expected {config.startup_altitude_m:.2f}m, '
        f'last altitude={last_altitude}'
    )


async def _own_altitude_or_none(drone) -> Optional[float]:
    if not hasattr(drone, 'position_ned'):
        return None
    try:
        position = await drone.position_ned()
    except AttributeError:
        return None
    return -position.down_m


def is_alignment_ready(
    observation: Optional[VisualObservation],
    config: StartupConfig,
    controller_config: Optional[FollowerControllerConfig] = None,
    current_time: Optional[float] = None,
) -> bool:
    if observation is None or not observation.target_visible:
        return False

    now = time.monotonic() if current_time is None else current_time
    if now - observation.timestamp > config.alignment_observation_timeout_s:
        return False

    mission_state, known_state = normalize_mission_state(observation.mission_state)
    if not known_state or mission_state != MissionState.FOLLOW:
        return False

    if not _valid_geometry(observation):
        return False

    follower_config = controller_config or FollowerControllerConfig.stable()
    size_error = follower_config.desired_target_size - observation.target_size
    return (
        abs(observation.horizontal_angle_deg) <= config.alignment_horizontal_tolerance_deg
        and abs(observation.vertical_angle_deg) <= config.alignment_vertical_tolerance_deg
        and abs(size_error) <= config.alignment_size_tolerance
    )


def alignment_command(
    observation: Optional[VisualObservation],
    startup_config: StartupConfig,
    controller_config: Optional[FollowerControllerConfig] = None,
) -> FollowerCommand:
    follower_config = controller_config or FollowerControllerConfig.stable()

    if not _valid_alignment_observation(observation):
        return FollowerCommand(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=0.0,
            yaw_rate_deg_s=_clamp(
                startup_config.alignment_search_yaw_rate,
                startup_config.alignment_max_yaw_rate,
            ),
            relay_state=MissionState.HOLD,
            state=FollowerState.SEARCH,
        )

    horizontal_angle = observation.horizontal_angle_deg
    vertical_angle = observation.vertical_angle_deg
    size_error = follower_config.desired_target_size - observation.target_size

    yaw_rate = follower_config.yaw_sign * follower_config.kp_yaw * horizontal_angle
    down = follower_config.vertical_sign * follower_config.kp_vertical * vertical_angle
    forward = follower_config.kp_forward * size_error

    return FollowerCommand(
        forward_m_s=_clamp(forward, startup_config.alignment_max_forward_speed),
        right_m_s=0.0,
        down_m_s=_clamp(down, startup_config.alignment_max_vertical_speed),
        yaw_rate_deg_s=_clamp(yaw_rate, startup_config.alignment_max_yaw_rate),
        relay_state=MissionState.HOLD,
        state=FollowerState.SEARCH,
        horizontal_angle_deg=horizontal_angle,
        vertical_angle_deg=vertical_angle,
        size_error=size_error,
    )


async def align_follower_to_target(
    follower,
    observation_provider: Callable[[], ObservationResult],
    follower_id: str,
    target_id: str,
    config: Optional[StartupConfig] = None,
    controller_config: Optional[FollowerControllerConfig] = None,
) -> AlignmentResult:
    startup_config = config or StartupConfig()
    follower_config = controller_config or FollowerControllerConfig.stable()
    actuator = DroneFollowerActuator(follower)
    deadline = time.monotonic() + startup_config.alignment_timeout_s
    ready_frames = 0
    last_command: Optional[FollowerCommand] = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        observation = observation_provider()
        if asyncio.iscoroutine(observation):
            observation = await observation

        if is_alignment_ready(observation, startup_config, follower_config, now):
            ready_frames += 1
        else:
            ready_frames = 0

        if ready_frames >= max(1, startup_config.alignment_required_frames):
            await actuator.safe_stop()
            return AlignmentResult(
                follower_id=follower_id,
                target_id=target_id,
                status=AlignmentStatus.READY,
                ready_frames=ready_frames,
                last_command=last_command,
            )

        last_command = alignment_command(observation, startup_config, follower_config)
        await actuator.apply(last_command, current_time=now)
        await asyncio.sleep(startup_config.startup_poll_interval_s)

    await actuator.safe_stop()
    return AlignmentResult(
        follower_id=follower_id,
        target_id=target_id,
        status=AlignmentStatus.FAILED,
        ready_frames=ready_frames,
        last_command=last_command,
        error=f'alignment timeout for {follower_id} -> {target_id}',
    )


async def align_chain_sequentially(
    followers: Sequence,
    observation_providers: Sequence[Callable[[], ObservationResult]],
    config: Optional[StartupConfig] = None,
    controller_config: Optional[FollowerControllerConfig] = None,
) -> list[AlignmentResult]:
    if len(followers) != len(observation_providers):
        raise StartupError('followers and observation_providers lengths must match')

    startup_config = config or StartupConfig()
    follower_config = controller_config or FollowerControllerConfig.stable()
    links = build_chain_config(len(followers))
    results: list[AlignmentResult] = []

    for follower, provider, link in zip(followers, observation_providers, links):
        result = await align_follower_to_target(
            follower,
            provider,
            link.follower_id,
            link.target_id,
            startup_config,
            follower_config,
        )
        results.append(result)
        if result.status != AlignmentStatus.READY:
            await safe_stop_all(followers)
            raise StartupError(result.error or f'alignment failed for {link.follower_id}')

    return results


async def wait_for_all_ready_then_start(
    all_ready_event: asyncio.Event,
    leader_mission: Callable[[], Awaitable[None]],
) -> None:
    await all_ready_event.wait()
    await leader_mission()


def drone_id(drone) -> str:
    return str(getattr(drone, 'drone_id', id(drone)))


def _valid_alignment_observation(observation: Optional[VisualObservation]) -> bool:
    if observation is None or not observation.target_visible:
        return False
    mission_state, known_state = normalize_mission_state(observation.mission_state)
    return known_state and mission_state == MissionState.FOLLOW and _valid_geometry(observation)


def _valid_geometry(observation: VisualObservation) -> bool:
    return (
        math.isfinite(observation.horizontal_angle_deg)
        and math.isfinite(observation.vertical_angle_deg)
        and math.isfinite(observation.target_size)
        and observation.target_size >= 0.0
    )


def _clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    if limit == 0.0:
        return 0.0
    return max(-limit, min(limit, value))
