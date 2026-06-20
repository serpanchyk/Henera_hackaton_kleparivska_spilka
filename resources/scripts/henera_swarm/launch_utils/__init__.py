"""Shared launch helpers for PX4/Gazebo swarm startup."""

from .formation import SpawnPose, TrainFormationConfig, train_positions

__all__ = [
    'SpawnPose',
    'TrainFormationConfig',
    'px4_instance',
    'train_positions',
]


def __getattr__(name: str):
    if name == 'px4_instance':
        from .px4_spawn import px4_instance

        return px4_instance
    raise AttributeError(f"module 'henera_swarm.launch_utils' has no attribute {name!r}")
