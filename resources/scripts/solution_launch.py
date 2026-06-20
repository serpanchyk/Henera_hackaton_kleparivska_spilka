import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)
_REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPTS_DIR))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from henera_swarm.launch_utils import (
    TrainFormationConfig,
    px4_instance,
    train_positions,
)
from drone_sdk.config import CONFIG

FOLLOWER_COUNT = CONFIG.runtime.follower_count
TRAIN_YAW_RAD = CONFIG.formation.train_yaw_rad
SPAWN_Z_M = CONFIG.formation.spawn_z_m
TRAIN_SPACING_M = CONFIG.formation.train_spacing_m


def leader_mission_process():
    return ExecuteProcess(
        cmd=["python3", os.path.join(SCRIPTS_DIR, "missions", "straight_line", "mission_launch.py")],
        output="screen",
    )


def follower_process(drone_id):
    return ExecuteProcess(
        cmd=["python3", os.path.join(SCRIPTS_DIR, "follower.py"), "--id", str(drone_id)],
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

    # t=5s: drones 1,2,3 — attach to running Gazebo
    for pose in poses[1:]:
        actions.append(TimerAction(
            period=5.0,
            actions=[px4_instance(pose.instance_id, pose.x, pose.y, pose.z, pose.yaw_rad)],
        ))

    # t=20s: leader mission
    actions.append(TimerAction(
        period=20.0,
        actions=[leader_mission_process()],
    ))

    # t=22s: follower scripts for configured follower drones
    for drone_id in range(1, FOLLOWER_COUNT + 1):
        actions.append(TimerAction(
            period=22.0,
            actions=[follower_process(drone_id)],
        ))

    return LaunchDescription(actions)
