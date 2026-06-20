"""
CV end-to-end launch (legal Path B) — real camera + LED optical channel.

Brings up ONLY the 4 PX4 SITL instances (Gazebo), then runs cv_swarm_test.py
which flies the leader (route + LED beacon) and the camera-based follower chain
in a single process. Does NOT launch mission_launch.py or follower.py.

PREREQUISITE: `bash project_setup.sh` (2-lens model in PX4).
"""
import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from henera_swarm.launch_utils import (
    TrainFormationConfig,
    px4_instance,
    train_positions,
)

# Swarm size + train formation come from config.yaml so the drones SPAWNED here
# match the count the control script (cv_swarm_test.py) CONNECTS to. Without this
# they drift: bumping follower_count alone leaves the controller waiting on
# drones that were never launched.
_REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPTS_DIR))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from drone_sdk.config import CONFIG

FOLLOWER_COUNT = CONFIG.runtime.follower_count
TRAIN_YAW_RAD = CONFIG.formation.train_yaw_rad
SPAWN_Z_M = CONFIG.formation.spawn_z_m
TRAIN_SPACING_M = CONFIG.formation.train_spacing_m


def cv_test_process():
    return ExecuteProcess(
        cmd=["python3", os.path.join(SCRIPTS_DIR, "cv_swarm_test.py")],
        output="screen",
    )


def generate_launch_description():
    actions = []
    poses = train_positions(
        TrainFormationConfig(
            leader_x=127.0,
            leader_y=52.67,
            z=SPAWN_Z_M,
            yaw_rad=TRAIN_YAW_RAD,
            spacing_m=TRAIN_SPACING_M,
            follower_count=FOLLOWER_COUNT,
        )
    )

    # t=0s: drone 0 (leader) — starts Gazebo
    leader = poses[0]
    actions.append(px4_instance(leader.instance_id, leader.x, leader.y, leader.z, leader.yaw_rad))

    # t=5s: followers — attach to running Gazebo
    for pose in poses[1:]:
        actions.append(TimerAction(
            period=5.0,
            actions=[px4_instance(pose.instance_id, pose.x, pose.y, pose.z, pose.yaw_rad)],
        ))

    # t=25s: single-process CV orchestrator (waits another 15s for EKF itself)
    actions.append(TimerAction(
        period=25.0,
        actions=[cv_test_process()],
    ))

    return LaunchDescription(actions)
