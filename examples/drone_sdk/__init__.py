from typing import TYPE_CHECKING

from .exceptions import (
    DroneSDKError,
    ConnectionError,
    TimeoutError,
    MAVSDKError,
    GazeboError,
    CameraError,
    LEDError,
)
from .follower_controller import (
    MissionState,
    FollowerState,
    VisualObservation,
    FollowerControllerConfig,
    FollowerStartupConfig,
    FollowerStartupError,
    FollowerCommand,
    ChainLinkConfig,
    FollowerController,
    MockVisualProvider,
    DroneFollowerActuator,
    build_chain_config,
    normalize_mission_state,
    prepare_followers_for_chain,
    safe_stop_all,
    run_follower_controller,
)
from .swarm_startup import (
    StartupError,
    StartupState,
    AlignmentStatus,
    StartupConfig,
    DroneStartupStatus,
    AlignmentResult,
    SwarmStartupCoordinator,
    prepare_swarm_for_start,
    safe_stop_all,
    align_follower_to_target,
    align_chain_sequentially,
    is_alignment_ready,
    alignment_command,
    wait_for_all_ready_then_start,
)

if TYPE_CHECKING:
    from .drone import Drone, PositionNED

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

        values = {
            'Drone': Drone,
            'PositionNED': PositionNED,
        }
        return values[name]
    raise AttributeError(f"module 'drone_sdk' has no attribute {name!r}")
