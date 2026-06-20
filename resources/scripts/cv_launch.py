"""
CV end-to-end launch (legal Path B) — real camera + LED optical channel.

Brings up ONLY the 4 PX4 SITL instances (Gazebo), then runs cv_swarm_test.py
which flies the leader (route + LED beacon) and the camera-based follower chain
in a single process. Does NOT launch mission_launch.py or follower.py.

PREREQUISITE: `bash project_setup.sh` (2-lens model in PX4).
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

from henera_swarm.launch_utils import (
    TrainFormationConfig,
    px4_instance,
    train_positions,
)

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
TRAIN_YAW_RAD = 3.7346
SPAWN_Z_M = 1.4
TRAIN_SPACING_M = 4.0


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
            follower_count=3,
        )
    )

    # t=0s: drone 0 (leader) — starts Gazebo
    leader = poses[0]
    actions.append(px4_instance(leader.instance_id, leader.x, leader.y, leader.z, leader.yaw_rad))

    # t=5s: drones 1,2,3 — attach to running Gazebo
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
