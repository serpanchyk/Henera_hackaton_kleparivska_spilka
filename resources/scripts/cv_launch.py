"""
CV end-to-end launch (legal Path B) — real camera + LED optical channel.

Brings up ONLY the 4 PX4 SITL instances (Gazebo), then runs cv_swarm_test.py
which flies the leader (route + LED beacon) and the camera-based follower chain
in a single process. Does NOT launch mission_launch.py or follower.py.

PREREQUISITE: `bash project_setup.sh` (2-lens model in PX4).
"""
import os
import math
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
TRAIN_YAW_RAD = 3.7346
SPAWN_Z_M = 1.4
TRAIN_SPACING_M = 4.0


def px4_instance(instance_id, x, y, z, yaw=3.7346):
    # PX4_GZ_STANDALONE=1: attach to the Gazebo server that start_cv.sh already
    # pre-started instead of letting each PX4 instance spawn its OWN server. Two
    # competing servers got one SIGKILLed and left the bridges timing out on /clock.
    cmd = f"""
        export DISPLAY=:0
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
        export GZ_RENDER_ENGINE=ogre2
        cd ~/PX4-Autopilot/ &&
        PX4_SYS_AUTOSTART=4010 \
        PX4_SIM_MODEL=gz_x500_mono_cam \
        PX4_GZ_WORLD=baylands_custom \
        PX4_GZ_STANDALONE=1 \
        PX4_GZ_MODEL_POSE="{x},{y},{z},0,0,{yaw}" \
        ./build/px4_sitl_default/bin/px4 -i {instance_id}
        """
    return ExecuteProcess(cmd=["bash", "-c", cmd], output="screen")


def cv_test_process():
    return ExecuteProcess(
        cmd=["python3", os.path.join(SCRIPTS_DIR, "cv_swarm_test.py")],
        output="screen",
    )


def generate_launch_description():
    actions = []
    # Train formation:
    # - yaw is the train direction; all drones face the same way so cameras look forward.
    # - followers are spawned behind the previous drone along the opposite yaw vector.
    # - 4.0 m spacing gives the CV controller room to correct without collisions.
    # - poses are always x,y,z,0,0,yaw to keep drones upright.
    leader_x = 127.0
    leader_y = 52.67
    behind_dx = -TRAIN_SPACING_M * math.cos(TRAIN_YAW_RAD)
    behind_dy = -TRAIN_SPACING_M * math.sin(TRAIN_YAW_RAD)

    # t=0s: drone 0 (leader) — attaches to the Gazebo server pre-started by start_cv.sh
    actions.append(px4_instance(0, leader_x, leader_y, SPAWN_Z_M, TRAIN_YAW_RAD))

    # t=5s: drones 1,2,3 — attach to running Gazebo
    follower_positions = [
        (1, leader_x + behind_dx, leader_y + behind_dy, SPAWN_Z_M),
        (2, leader_x + 2 * behind_dx, leader_y + 2 * behind_dy, SPAWN_Z_M),
        (3, leader_x + 3 * behind_dx, leader_y + 3 * behind_dy, SPAWN_Z_M),
    ]
    for idx, x, y, z in follower_positions:
        actions.append(TimerAction(
            period=5.0,
            actions=[px4_instance(idx, x, y, z)],
        ))

    # t=25s: single-process CV orchestrator (waits another 15s for EKF itself)
    actions.append(TimerAction(
        period=25.0,
        actions=[cv_test_process()],
    ))

    return LaunchDescription(actions)
