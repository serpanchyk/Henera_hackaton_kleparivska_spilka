import math
import os
import sys
from dataclasses import dataclass

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from drone_sdk.config import CONFIG


@dataclass(frozen=True)
class SpawnPose:
    instance_id: int
    x: float
    y: float
    z: float
    yaw_rad: float


@dataclass(frozen=True)
class TrainFormationConfig:
    leader_x: float = 127.0
    leader_y: float = 52.67
    z: float = CONFIG.formation.spawn_z_m
    yaw_rad: float = CONFIG.formation.train_yaw_rad
    spacing_m: float = CONFIG.formation.train_spacing_m
    follower_count: int = CONFIG.runtime.follower_count


def train_positions(config: TrainFormationConfig) -> list[SpawnPose]:
    if config.follower_count < 0:
        raise ValueError('follower_count must be non-negative')

    behind_dx = -config.spacing_m * math.cos(config.yaw_rad)
    behind_dy = -config.spacing_m * math.sin(config.yaw_rad)
    poses = [
        SpawnPose(
            instance_id=0,
            x=config.leader_x,
            y=config.leader_y,
            z=config.z,
            yaw_rad=config.yaw_rad,
        )
    ]
    for index in range(1, config.follower_count + 1):
        poses.append(
            SpawnPose(
                instance_id=index,
                x=config.leader_x + index * behind_dx,
                y=config.leader_y + index * behind_dy,
                z=config.z,
                yaw_rad=config.yaw_rad,
            )
        )
    return poses
