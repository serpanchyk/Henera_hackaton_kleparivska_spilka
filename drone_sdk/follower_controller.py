import asyncio
import math
import sys
import time
from dataclasses import dataclass, replace
from enum import Enum
from typing import Awaitable, Callable, Iterable, Optional

try:
    from .config import CONFIG
    from .swarm_speeds import (
        FOLLOWER_MAX_FORWARD_SPEED_M_S,
        FOLLOWER_MAX_REVERSE_SPEED_M_S,
        LEADER_CRUISE_SPEED_M_S,
        follower_forward_for,
        follower_reverse_for,
    )
except ImportError:
    # Loaded standalone (e.g. the unit tests import this file by path, without
    # the drone_sdk package). config.py / swarm_speeds.py are dependency-light,
    # so pull them from the same directory.
    import os as _os
    sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
    from config import CONFIG
    from swarm_speeds import (
        FOLLOWER_MAX_FORWARD_SPEED_M_S,
        FOLLOWER_MAX_REVERSE_SPEED_M_S,
        LEADER_CRUISE_SPEED_M_S,
        follower_forward_for,
        follower_reverse_for,
    )

_FC = CONFIG.follower_control


class MissionState(str, Enum):
    FOLLOW = 'FOLLOW'
    HOLD = 'HOLD'
    FINISH = 'FINISH'
    SAFE = 'SAFE'


class FollowerState(str, Enum):
    SEARCH = 'SEARCH'
    FOLLOW = 'FOLLOW'
    LOST = 'LOST'
    HOLD = 'HOLD'
    FINISH = 'FINISH'


class FollowerStartupError(RuntimeError):
    pass


@dataclass(frozen=True)
class VisualObservation:
    target_visible: bool
    horizontal_angle_deg: float
    vertical_angle_deg: float
    target_size: float
    mission_state: MissionState | str
    timestamp: float


@dataclass(frozen=True)
class FollowerControllerConfig:
    # Defaults are sourced from config.yaml (the `follower_control` section) so
    # there is a single place to retune the controller. Forward/reverse caps
    # come from the `speeds` section (matched to the leader cruise) so a
    # follower never dramatically outruns the leader and, when it gets too
    # close, eases back gently instead of slamming into the trailing follower.
    desired_target_size: float = _FC.desired_target_size
    kp_yaw: float = _FC.kp_yaw
    kp_forward: float = _FC.kp_forward
    kp_vertical: float = _FC.kp_vertical
    max_yaw_rate: float = _FC.max_yaw_rate
    max_forward_speed: float = FOLLOWER_MAX_FORWARD_SPEED_M_S
    max_reverse_speed: float = FOLLOWER_MAX_REVERSE_SPEED_M_S
    max_vertical_speed: float = _FC.max_vertical_speed
    yaw_dead_zone_deg: float = _FC.yaw_dead_zone_deg
    vertical_dead_zone_deg: float = _FC.vertical_dead_zone_deg
    size_dead_zone: float = _FC.size_dead_zone
    lost_timeout: float = _FC.lost_timeout
    observation_timeout: float = _FC.observation_timeout
    lost_frames_threshold: int = _FC.lost_frames_threshold
    reacquire_frames: int = _FC.reacquire_frames
    smoothing_alpha: float = _FC.smoothing_alpha
    search_yaw_rate: float = _FC.search_yaw_rate  # legacy; superseded by the scan sweep
    # SEARCH scan: instead of spinning a full 360 (which re-acquires the wrong drone,
    # e.g. our own follower), sweep a bounded window around the heading where the
    # target was lost. Left/right is a true yaw sweep; up/down is an altitude bob
    # (the camera is body-fixed, so vertical "look" is realized by changing height).
    search_yaw_sweep_deg: float = _FC.search_yaw_sweep_deg
    search_vertical_speed: float = _FC.search_vertical_speed
    search_period_s: float = _FC.search_period_s
    control_rate_hz: float = _FC.control_rate_hz
    yaw_sign: float = _FC.yaw_sign
    vertical_sign: float = _FC.vertical_sign
    lost_command_memory_s: float = _FC.lost_command_memory_s

    @classmethod
    def matched(
        cls,
        leader_cruise_m_s: float = LEADER_CRUISE_SPEED_M_S,
        **overrides,
    ) -> 'FollowerControllerConfig':
        """Build a config whose speed caps are matched to the leader's cruise.

        This is the canonical way for swarm scripts to size follower speed:
        pass the same leader cruise the leader is flying and the forward/reverse
        caps follow automatically, so the two can never drift into the
        crowd-and-reverse mismatch. Any field can still be overridden by keyword.
        """
        base = dict(
            max_forward_speed=follower_forward_for(leader_cruise_m_s),
            max_reverse_speed=follower_reverse_for(leader_cruise_m_s),
        )
        base.update(overrides)
        return cls(**base)

    @classmethod
    def stable(cls) -> 'FollowerControllerConfig':
        return cls()

    @classmethod
    def responsive(cls) -> 'FollowerControllerConfig':
        return cls(
            control_rate_hz=20.0,
            reacquire_frames=2,
            observation_timeout=0.25,
            smoothing_alpha=0.2,
            kp_yaw=1.5,
            kp_forward=0.03,
            kp_vertical=0.03,
        )


@dataclass(frozen=True)
class FollowerStartupConfig:
    startup_altitude_m: float = 10.0
    startup_timeout_s: float = 30.0
    startup_settle_time_s: float = 2.0
    startup_height_tolerance_m: float = 0.75
    startup_poll_interval_s: float = 0.25


@dataclass(frozen=True)
class FollowerCommand:
    forward_m_s: float
    right_m_s: float
    down_m_s: float
    yaw_rate_deg_s: float
    relay_state: MissionState
    state: FollowerState
    horizontal_angle_deg: Optional[float] = None
    vertical_angle_deg: Optional[float] = None
    size_error: Optional[float] = None


@dataclass(frozen=True)
class ChainLinkConfig:
    follower_id: str
    target_id: str
    drone_id: int


def normalize_mission_state(value: MissionState | str) -> tuple[MissionState, bool]:
    if isinstance(value, MissionState):
        return value, True
    if isinstance(value, str):
        try:
            return MissionState(value.upper()), True
        except ValueError:
            return MissionState.HOLD, False
    return MissionState.HOLD, False


MIN_FOLLOWERS = 2
# Sanity ceiling only — guards against a config typo (e.g. follower_count: 1000),
# not a real engineering limit. Raise it if you genuinely need a longer chain.
MAX_FOLLOWERS = 20


def build_chain_config(follower_count: int) -> list[ChainLinkConfig]:
    if follower_count < MIN_FOLLOWERS or follower_count > MAX_FOLLOWERS:
        raise ValueError(
            f'follower_count must be in range {MIN_FOLLOWERS}..{MAX_FOLLOWERS}'
        )
    return [
        ChainLinkConfig(
            follower_id=f'follower_{index}',
            target_id='leader' if index == 1 else f'follower_{index - 1}',
            drone_id=index,
        )
        for index in range(1, follower_count + 1)
    ]


class FollowerController:

    def __init__(
        self,
        follower_id: str,
        target_id: str,
        config: Optional[FollowerControllerConfig] = None,
    ):
        self.follower_id = follower_id
        self.target_id = target_id
        self.config = config or FollowerControllerConfig()
        self.state = FollowerState.SEARCH
        self._valid_frames = 0
        self._lost_frames = 0
        self._lost_since: Optional[float] = None
        self._search_started_at: Optional[float] = None
        self._last_command = self._zero_command(FollowerState.SEARCH, MissionState.HOLD)

    def reset(self) -> None:
        self.state = FollowerState.SEARCH
        self._valid_frames = 0
        self._lost_frames = 0
        self._lost_since = None
        self._search_started_at = None
        self._last_command = self._zero_command(FollowerState.SEARCH, MissionState.HOLD)

    def update(
        self,
        observation: Optional[VisualObservation],
        current_time: Optional[float] = None,
    ) -> FollowerCommand:
        now = time.monotonic() if current_time is None else current_time

        # Anchor the scan clock the moment we (re)enter SEARCH so each search starts
        # centered on the lost heading rather than continuing a stale sweep phase.
        if self.state != FollowerState.SEARCH:
            self._search_started_at = None

        if self.state == FollowerState.FINISH:
            self._last_command = self._finish_command()
            return self._last_command

        mission_state, known_state = self._mission_state(observation)
        if mission_state == MissionState.FINISH:
            self.state = FollowerState.FINISH
            self._last_command = self._finish_command()
            return self._last_command

        if mission_state == MissionState.SAFE:
            self.state = FollowerState.HOLD
            self._valid_frames = 0
            self._lost_frames = 0
            self._lost_since = None
            self._last_command = self._zero_command(FollowerState.HOLD, MissionState.SAFE)
            return self._last_command

        if not known_state or mission_state == MissionState.HOLD:
            self.state = FollowerState.HOLD
            self._valid_frames = 0
            self._lost_frames = 0
            self._lost_since = None
            self._last_command = self._zero_command(FollowerState.HOLD, MissionState.HOLD)
            return self._last_command

        valid_follow = self._is_valid_follow_observation(observation, now)
        if valid_follow:
            return self._handle_valid_follow(observation, now)

        return self._handle_missing_or_invalid(observation, now)

    def _handle_valid_follow(self, observation: VisualObservation, now: float) -> FollowerCommand:
        self._valid_frames += 1
        self._lost_frames = 0
        self._lost_since = None

        if self._valid_frames < max(1, self.config.reacquire_frames):
            if self.state == FollowerState.HOLD:
                self._last_command = self._zero_command(FollowerState.HOLD, MissionState.HOLD)
            elif self.state == FollowerState.LOST:
                self._last_command = self._lost_command(observation)
            else:
                self.state = FollowerState.SEARCH
                self._last_command = self._search_command(observation, now)
            return self._last_command

        self.state = FollowerState.FOLLOW
        self._last_command = self._follow_command(observation)
        return self._last_command

    def _handle_missing_or_invalid(
        self,
        observation: Optional[VisualObservation],
        now: float,
    ) -> FollowerCommand:
        self._valid_frames = 0

        if self.state == FollowerState.SEARCH:
            self._last_command = self._search_command(observation, now)
            return self._last_command

        if self.state == FollowerState.HOLD:
            self.state = FollowerState.SEARCH
            self._last_command = self._search_command(observation, now)
            return self._last_command

        if self.state == FollowerState.FOLLOW:
            self._lost_frames += 1
            if self._lost_frames < max(1, self.config.lost_frames_threshold):
                self._last_command = self._zero_forward_command(observation)
                return self._last_command
            self.state = FollowerState.LOST
            self._lost_since = now

        if self.state == FollowerState.LOST:
            if self._lost_since is None:
                self._lost_since = now
            if now - self._lost_since >= self.config.lost_timeout:
                self.state = FollowerState.SEARCH
                self._last_command = self._search_command(observation)
            else:
                self._last_command = self._lost_command(observation, now)
            return self._last_command

        self.state = FollowerState.SEARCH
        self._last_command = self._search_command(observation, now)
        return self._last_command

    def _mission_state(
        self,
        observation: Optional[VisualObservation],
    ) -> tuple[MissionState, bool]:
        if observation is None:
            return MissionState.FOLLOW, True
        return normalize_mission_state(observation.mission_state)

    def _is_valid_follow_observation(
        self,
        observation: Optional[VisualObservation],
        now: float,
    ) -> bool:
        if observation is None or not observation.target_visible:
            return False
        if now - observation.timestamp > self.config.observation_timeout:
            return False
        if not math.isfinite(observation.horizontal_angle_deg):
            return False
        if not math.isfinite(observation.vertical_angle_deg):
            return False
        if not math.isfinite(observation.target_size) or observation.target_size < 0:
            return False
        mission_state, known_state = normalize_mission_state(observation.mission_state)
        return known_state and mission_state == MissionState.FOLLOW

    def _follow_command(self, observation: VisualObservation) -> FollowerCommand:
        horizontal_angle = observation.horizontal_angle_deg
        vertical_angle = observation.vertical_angle_deg
        size_error = self.config.desired_target_size - observation.target_size

        yaw_rate = 0.0
        if abs(horizontal_angle) >= self.config.yaw_dead_zone_deg:
            yaw_rate = self.config.yaw_sign * self.config.kp_yaw * horizontal_angle

        down = 0.0
        if abs(vertical_angle) >= self.config.vertical_dead_zone_deg:
            down = self.config.vertical_sign * self.config.kp_vertical * vertical_angle

        forward = 0.0
        if abs(size_error) >= self.config.size_dead_zone:
            forward = self.config.kp_forward * size_error

        # Asymmetric clamp: approach up to max_forward_speed, but when the target
        # looks too big (size_error < 0 -> too close) back off only at the much
        # smaller max_reverse_speed. A symmetric clamp let a crowded follower
        # reverse at full speed, which produced the dangerous fly-back into the
        # trailing drone.
        forward = self._clamp_forward(forward)

        command = FollowerCommand(
            forward_m_s=forward,
            right_m_s=0.0,
            down_m_s=self._clamp(down, self.config.max_vertical_speed),
            yaw_rate_deg_s=self._clamp(yaw_rate, self.config.max_yaw_rate),
            relay_state=MissionState.FOLLOW,
            state=FollowerState.FOLLOW,
            horizontal_angle_deg=horizontal_angle,
            vertical_angle_deg=vertical_angle,
            size_error=size_error,
        )
        return self._smooth(command)

    def _search_command(
        self,
        observation: Optional[VisualObservation],
        now: Optional[float] = None,
    ) -> FollowerCommand:
        # Bounded scan instead of a full 360 spin: sweep yaw left/right within
        # +/- search_yaw_sweep_deg and bob altitude up/down, both centered on the
        # heading/height where the target was lost. cos() keeps yaw at its extreme
        # rate when the scan starts (so it begins turning immediately) while sin()
        # keeps the vertical bob at zero at the start (no jump in altitude).
        if now is None:
            now = time.monotonic()
        if self._search_started_at is None:
            self._search_started_at = now
        tau = max(0.0, now - self._search_started_at)

        omega = 2.0 * math.pi / max(0.1, self.config.search_period_s)
        yaw_rate = (
            self.config.yaw_sign
            * self.config.search_yaw_sweep_deg
            * omega
            * math.cos(omega * tau)
        )
        vertical = (
            self.config.vertical_sign
            * self.config.search_vertical_speed
            * math.sin(omega * tau)
        )
        return FollowerCommand(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=self._clamp(vertical, self.config.max_vertical_speed),
            yaw_rate_deg_s=self._clamp(yaw_rate, self.config.max_yaw_rate),
            relay_state=MissionState.HOLD,
            state=FollowerState.SEARCH,
            horizontal_angle_deg=self._finite_field(observation, 'horizontal_angle_deg'),
            vertical_angle_deg=self._finite_field(observation, 'vertical_angle_deg'),
            size_error=None,
        )

    def _lost_command(self, observation: Optional[VisualObservation], now: Optional[float] = None) -> FollowerCommand:
        keep_previous = (
            self.config.lost_command_memory_s > 0.0
            and now is not None
            and self._lost_since is not None
            and now - self._lost_since <= self.config.lost_command_memory_s
        )
        alpha = self._bounded_alpha() if keep_previous else 0.0
        return FollowerCommand(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=self._clamp(self._last_command.down_m_s * alpha, self.config.max_vertical_speed),
            yaw_rate_deg_s=self._clamp(self._last_command.yaw_rate_deg_s * alpha, self.config.max_yaw_rate),
            relay_state=MissionState.HOLD,
            state=FollowerState.LOST,
            horizontal_angle_deg=self._finite_field(observation, 'horizontal_angle_deg'),
            vertical_angle_deg=self._finite_field(observation, 'vertical_angle_deg'),
            size_error=None,
        )

    def _zero_forward_command(self, observation: Optional[VisualObservation]) -> FollowerCommand:
        return FollowerCommand(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=0.0,
            yaw_rate_deg_s=0.0,
            relay_state=MissionState.HOLD,
            state=self.state,
            horizontal_angle_deg=self._finite_field(observation, 'horizontal_angle_deg'),
            vertical_angle_deg=self._finite_field(observation, 'vertical_angle_deg'),
            size_error=None,
        )

    def _zero_command(
        self,
        state: FollowerState,
        relay_state: MissionState,
    ) -> FollowerCommand:
        return FollowerCommand(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=0.0,
            yaw_rate_deg_s=0.0,
            relay_state=relay_state,
            state=state,
        )

    def _finish_command(self) -> FollowerCommand:
        return self._zero_command(FollowerState.FINISH, MissionState.FINISH)

    def _smooth(self, command: FollowerCommand) -> FollowerCommand:
        last = self._last_command
        if last.state != FollowerState.FOLLOW:
            return command
        alpha = self._bounded_alpha()
        return FollowerCommand(
            forward_m_s=self._blend(last.forward_m_s, command.forward_m_s, alpha),
            right_m_s=0.0,
            down_m_s=self._blend(last.down_m_s, command.down_m_s, alpha),
            yaw_rate_deg_s=self._blend(last.yaw_rate_deg_s, command.yaw_rate_deg_s, alpha),
            relay_state=command.relay_state,
            state=command.state,
            horizontal_angle_deg=command.horizontal_angle_deg,
            vertical_angle_deg=command.vertical_angle_deg,
            size_error=command.size_error,
        )

    def _bounded_alpha(self) -> float:
        return max(0.0, min(1.0, self.config.smoothing_alpha))

    @staticmethod
    def _blend(previous: float, current: float, alpha: float) -> float:
        return previous * alpha + current * (1.0 - alpha)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        limit = abs(limit)
        if limit == 0.0:
            return 0.0
        return max(-limit, min(limit, value))

    def _clamp_forward(self, value: float) -> float:
        """Clamp forward velocity asymmetrically: full speed forward, gentle reverse.

        Positive = approach (capped at max_forward_speed); negative = back off
        because the target is too close (capped at the smaller max_reverse_speed).
        """
        if value >= 0.0:
            return min(value, abs(self.config.max_forward_speed))
        return max(value, -abs(self.config.max_reverse_speed))

    @staticmethod
    def _finite_field(
        observation: Optional[VisualObservation],
        field_name: str,
    ) -> Optional[float]:
        if observation is None:
            return None
        value = getattr(observation, field_name)
        return value if isinstance(value, (int, float)) and math.isfinite(value) else None


class MockVisualProvider:

    def __init__(
        self,
        observations: Iterable[VisualObservation],
        repeat_last: bool = True,
    ):
        self._observations = list(observations)
        self._repeat_last = repeat_last
        self._index = 0

    def next_observation(self, current_time: Optional[float] = None) -> Optional[VisualObservation]:
        if not self._observations:
            return None
        if self._index >= len(self._observations):
            if not self._repeat_last:
                return None
            observation = self._observations[-1]
        else:
            observation = self._observations[self._index]
            self._index += 1
        if current_time is None:
            return observation
        return VisualObservation(
            target_visible=observation.target_visible,
            horizontal_angle_deg=observation.horizontal_angle_deg,
            vertical_angle_deg=observation.vertical_angle_deg,
            target_size=observation.target_size,
            mission_state=observation.mission_state,
            timestamp=current_time if math.isfinite(observation.timestamp) else observation.timestamp,
        )

    @classmethod
    def normal_follow(
        cls,
        count: int,
        target_size: float = 70.0,
        horizontal_angle_deg: float = 5.0,
        vertical_angle_deg: float = 0.0,
        start_time: float = 0.0,
        dt: float = 0.1,
    ) -> 'MockVisualProvider':
        return cls(
            VisualObservation(
                True,
                horizontal_angle_deg,
                vertical_angle_deg,
                target_size,
                MissionState.FOLLOW,
                start_time + index * dt,
            )
            for index in range(count)
        )

    @classmethod
    def scenario(
        cls,
        names: Iterable[str],
        start_time: float = 0.0,
        dt: float = 0.1,
    ) -> 'MockVisualProvider':
        observations = []
        now = start_time
        for name in names:
            observations.append(cls._named_observation(name, now))
            now += dt
        return cls(observations, repeat_last=False)

    @staticmethod
    def _named_observation(name: str, timestamp: float) -> VisualObservation:
        if name == 'follow':
            return VisualObservation(True, 4.0, 1.0, 70.0, MissionState.FOLLOW, timestamp)
        if name == 'short_loss' or name == 'long_loss':
            return VisualObservation(False, 0.0, 0.0, 0.0, MissionState.FOLLOW, timestamp)
        if name == 'stale':
            return VisualObservation(True, 0.0, 0.0, 70.0, MissionState.FOLLOW, timestamp - 1000.0)
        if name == 'hold':
            return VisualObservation(True, 0.0, 0.0, 70.0, MissionState.HOLD, timestamp)
        if name == 'safe':
            return VisualObservation(True, 0.0, 0.0, 70.0, MissionState.SAFE, timestamp)
        if name == 'finish':
            return VisualObservation(True, 0.0, 0.0, 70.0, MissionState.FINISH, timestamp)
        if name == 'reacquire':
            return VisualObservation(True, 0.0, 0.0, 80.0, MissionState.FOLLOW, timestamp)
        raise ValueError(f'unknown mock scenario item: {name}')


class DroneFollowerActuator:

    def __init__(
        self,
        drone,
        yaw_update_dt: Optional[float] = None,
        max_yaw_integration_dt: float = 0.1,
    ):
        self.drone = drone
        self.yaw_update_dt = yaw_update_dt
        self.max_yaw_integration_dt = max_yaw_integration_dt
        self._last_apply_time: Optional[float] = None
        self._yaw_deg: Optional[float] = None

    async def apply(self, command: FollowerCommand, current_time: Optional[float] = None) -> None:
        now = time.monotonic() if current_time is None else current_time
        heading = await self.drone.heading()
        if self._yaw_deg is None:
            self._yaw_deg = heading
        dt = self.yaw_update_dt
        if dt is None:
            dt = 0.0 if self._last_apply_time is None else max(0.0, now - self._last_apply_time)
        dt = min(dt, self.max_yaw_integration_dt)
        self._last_apply_time = now
        self._yaw_deg = (self._yaw_deg + command.yaw_rate_deg_s * dt) % 360.0
        yaw_rad = math.radians(heading)
        north_m_s = command.forward_m_s * math.cos(yaw_rad) - command.right_m_s * math.sin(yaw_rad)
        east_m_s = command.forward_m_s * math.sin(yaw_rad) + command.right_m_s * math.cos(yaw_rad)
        try:
            await self.drone.set_velocity(
                north_m_s,
                east_m_s,
                command.down_m_s,
                yaw_deg=self._yaw_deg,
            )
        except Exception:
            await self.safe_stop(heading)
            raise

    async def safe_stop(self, heading: Optional[float] = None) -> None:
        if heading is None:
            heading = await self.drone.heading()
        await self.drone.set_velocity(0.0, 0.0, 0.0, yaw_deg=heading)


async def safe_stop_all(drones: Iterable) -> None:
    for drone in drones:
        try:
            heading = await drone.heading()
            await drone.set_velocity(0.0, 0.0, 0.0, yaw_deg=heading)
        except Exception:
            pass


async def _wait_until_startup_altitude(drone, config: FollowerStartupConfig) -> None:
    deadline = time.monotonic() + config.startup_timeout_s
    while time.monotonic() < deadline:
        position = await drone.position_ned()
        altitude_m = -position.down_m
        if abs(altitude_m - config.startup_altitude_m) <= config.startup_height_tolerance_m:
            return
        await asyncio.sleep(config.startup_poll_interval_s)
    raise FollowerStartupError(
        f'drone startup altitude timeout: expected {config.startup_altitude_m:.2f}m'
    )


async def prepare_followers_for_chain(
    drones: Iterable,
    config: Optional[FollowerStartupConfig] = None,
) -> None:
    startup_config = config or FollowerStartupConfig()
    prepared = list(drones)
    try:
        for drone in prepared:
            await drone.arm()
        await asyncio.gather(*(
            drone.takeoff(altitude_m=startup_config.startup_altitude_m)
            for drone in prepared
        ))
        await asyncio.gather(*(
            _wait_until_startup_altitude(drone, startup_config)
            for drone in prepared
        ))
        if startup_config.startup_settle_time_s > 0.0:
            await asyncio.sleep(startup_config.startup_settle_time_s)
        for drone in prepared:
            await drone.start_offboard()
        await safe_stop_all(prepared)
    except Exception as exc:
        await safe_stop_all(prepared)
        if isinstance(exc, FollowerStartupError):
            raise
        raise FollowerStartupError(f'follower startup failed: {exc}') from exc


async def run_follower_controller(
    controller: FollowerController,
    observation_provider: Callable[[], Optional[VisualObservation] | Awaitable[Optional[VisualObservation]]],
    actuator: DroneFollowerActuator,
    stop_condition: Callable[[], bool],
) -> None:
    period = 1.0 / controller.config.control_rate_hz
    while not stop_condition():
        observation = observation_provider()
        if asyncio.iscoroutine(observation):
            observation = await observation
        command = controller.update(observation)
        await actuator.apply(command)
        await asyncio.sleep(period)
