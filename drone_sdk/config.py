"""Typed loader for the central tunable configuration (``config.yaml``).

All tunable numbers live in ``config.yaml`` at the repo root. This module loads
them into frozen dataclasses so the rest of the code reads
``CONFIG.<section>.<field>`` instead of duplicating magic numbers.

We use plain dataclasses rather than pydantic on purpose: the SDK already models
its configs as dataclasses, and avoiding the extra runtime dependency keeps the
core import-light (the unit tests load modules standalone). Loading is strict —
unknown or missing keys raise immediately so a typo in ``config.yaml`` fails
loudly instead of silently falling back to a default.

Override the file with the ``FALCON_CONFIG`` environment variable.
"""

import os
from dataclasses import dataclass, fields, is_dataclass
from functools import lru_cache

import yaml


@dataclass(frozen=True)
class SpeedsConfig:
    leader_cruise_m_s: float
    follower_forward_ratio: float
    follower_reverse_ratio: float


@dataclass(frozen=True)
class FollowerControlConfig:
    desired_target_size: float
    kp_yaw: float
    kp_forward: float
    kp_vertical: float
    max_yaw_rate: float
    max_vertical_speed: float
    yaw_dead_zone_deg: float
    vertical_dead_zone_deg: float
    size_dead_zone: float
    lost_timeout: float
    observation_timeout: float
    lost_frames_threshold: int
    reacquire_frames: int
    smoothing_alpha: float
    search_yaw_rate: float
    search_yaw_sweep_deg: float
    search_vertical_speed: float
    search_period_s: float
    control_rate_hz: float
    yaw_sign: float
    vertical_sign: float
    lost_command_memory_s: float


@dataclass(frozen=True)
class PerceptionConfig:
    acquire_min_target_size: float
    acquire_max_target_size: float


@dataclass(frozen=True)
class FormationConfig:
    train_spacing_m: float
    spawn_z_m: float
    train_yaw_rad: float


@dataclass(frozen=True)
class RuntimeConfig:
    follower_count: int
    common_alt_m: float
    ekf_settle_s: float
    hover_settle_s: float
    takeoff_timeout_s: float
    takeoff_tolerance_m: float
    takeoff_airborne_fraction: float
    control_hz: float
    beacon_hz: float
    truth_log_hz: float
    watchdog_s: float
    finish_broadcast_s: float
    finish_descent_speed_ms: float
    finish_descent_timeout_s: float
    finish_ground_alt_m: float
    acquire_hold_s: float
    route_recovery_pause_s: float
    route_recovery_poll_s: float
    step_m: float


@dataclass(frozen=True)
class PortsConfig:
    mavsdk_udp_base: int
    mavsdk_grpc_base: int


@dataclass(frozen=True)
class Config:
    speeds: SpeedsConfig
    follower_control: FollowerControlConfig
    perception: PerceptionConfig
    formation: FormationConfig
    runtime: RuntimeConfig
    ports: PortsConfig


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG_PATH = os.path.join(_REPO_ROOT, 'config.yaml')


def _coerce(value, target_type, path):
    if target_type is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError(f"config {path!r}: expected a number, got {value!r}")
        return float(value)
    if target_type is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"config {path!r}: expected an integer, got {value!r}")
        return value
    return value


def _from_dict(cls, data, path=''):
    if not isinstance(data, dict):
        raise TypeError(f"config {path or 'root'!r}: expected a mapping, got {type(data).__name__}")
    expected = {f.name for f in fields(cls)}
    extra = set(data) - expected
    if extra:
        raise ValueError(f"config {path or 'root'!r}: unknown keys {sorted(extra)}")
    missing = expected - set(data)
    if missing:
        raise ValueError(f"config {path or 'root'!r}: missing keys {sorted(missing)}")
    kwargs = {}
    for f in fields(cls):
        child_path = f"{path}.{f.name}" if path else f.name
        raw = data[f.name]
        if is_dataclass(f.type):
            kwargs[f.name] = _from_dict(f.type, raw, child_path)
        else:
            kwargs[f.name] = _coerce(raw, f.type, child_path)
    return cls(**kwargs)


@lru_cache(maxsize=None)
def load_config(path: str = None) -> Config:
    """Load and validate ``config.yaml`` into a typed :class:`Config` (cached)."""
    path = path or os.environ.get('FALCON_CONFIG') or DEFAULT_CONFIG_PATH
    with open(path, 'r', encoding='utf-8') as fh:
        raw = yaml.safe_load(fh) or {}
    return _from_dict(Config, raw)


# Module-level singleton for convenient `from drone_sdk.config import CONFIG`.
CONFIG = load_config()
