"""Compatibility package that redirects to the public root ``drone_sdk``.

When an example is run as ``python3 examples/demo.py``, Python puts
``examples/`` first on ``sys.path``. This package keeps that launch mode
working while loading the real implementation from ``../drone_sdk``.
"""

from pathlib import Path

_ROOT_PACKAGE = Path(__file__).resolve().parents[2] / 'drone_sdk'
__path__ = [str(_ROOT_PACKAGE), *__path__]

from .exceptions import (  # noqa: E402
    CameraError,
    ConnectionError,
    DroneSDKError,
    GazeboError,
    LEDError,
    MAVSDKError,
    TimeoutError,
)
from .follower_controller import (  # noqa: E402
    ChainLinkConfig,
    DroneFollowerActuator,
    FollowerCommand,
    FollowerController,
    FollowerControllerConfig,
    FollowerStartupConfig,
    FollowerStartupError,
    FollowerState,
    MissionState,
    MockVisualProvider,
    VisualObservation,
    build_chain_config,
    normalize_mission_state,
    prepare_followers_for_chain,
    run_follower_controller,
    safe_stop_all,
)
from .swarm_startup import (  # noqa: E402
    AlignmentResult,
    AlignmentStatus,
    DroneStartupStatus,
    StartupConfig,
    StartupError,
    StartupState,
    SwarmStartupCoordinator,
    align_chain_sequentially,
    align_follower_to_target,
    alignment_command,
    is_alignment_ready,
    prepare_swarm_for_start,
    wait_for_all_ready_then_start,
)

__all__ = [
    'Drone',
    'PositionNED',
    'DroneSDKError',
    'ConnectionError',
    'TimeoutError',
    'MAVSDKError',
    'GazeboError',
    'CameraError',
    'LEDError',
    'MissionState',
    'FollowerState',
    'VisualObservation',
    'FollowerControllerConfig',
    'FollowerStartupConfig',
    'FollowerStartupError',
    'FollowerCommand',
    'ChainLinkConfig',
    'FollowerController',
    'MockVisualProvider',
    'DroneFollowerActuator',
    'build_chain_config',
    'normalize_mission_state',
    'prepare_followers_for_chain',
    'safe_stop_all',
    'run_follower_controller',
    'wait_for_all_ready_then_start',
    'alignment_command',
    'is_alignment_ready',
    'align_chain_sequentially',
    'align_follower_to_target',
    'prepare_swarm_for_start',
    'SwarmStartupCoordinator',
    'AlignmentResult',
    'DroneStartupStatus',
    'StartupConfig',
    'AlignmentStatus',
    'StartupState',
    'StartupError',
]


def __getattr__(name: str):
    if name in {'Drone', 'PositionNED'}:
        from .drone import Drone, PositionNED

        return {'Drone': Drone, 'PositionNED': PositionNED}[name]
    raise AttributeError(f"module 'drone_sdk' has no attribute {name!r}")
