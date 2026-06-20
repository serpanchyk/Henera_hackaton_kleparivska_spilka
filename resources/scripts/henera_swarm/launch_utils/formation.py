import math
from dataclasses import dataclass


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
    z: float = 1.4
    yaw_rad: float = 3.7346
    spacing_m: float = 2.0
    follower_count: int = 3


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
