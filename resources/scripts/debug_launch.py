"""
DEBUG / EVAL ONLY launch — ground-truth follower test.

Brings up ONLY the 4 PX4 SITL instances (Gazebo), then runs debug_swarm_test.py
which flies the leader and the follower chain in a single process. Does NOT
launch mission_launch.py or follower.py — the debug script owns leader+followers.
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))


def px4_instance(instance_id, x, y, z, yaw=3.7346):
    # Env vars are set explicitly so Gazebo gets them even in subprocess context
    cmd = f"""
        export DISPLAY=:0
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
        export GZ_RENDER_ENGINE=ogre2
        cd ~/PX4-Autopilot/ &&
        PX4_SYS_AUTOSTART=4010 \
        PX4_SIM_MODEL=gz_x500_mono_cam \
        PX4_GZ_WORLD=baylands_custom \
        PX4_GZ_MODEL_POSE="{x},{y},{z},0,0,{yaw}" \
        ./build/px4_sitl_default/bin/px4 -i {instance_id}
        """
    return ExecuteProcess(cmd=["bash", "-c", cmd], output="screen")


def debug_test_process():
    return ExecuteProcess(
        cmd=["python3", os.path.join(SCRIPTS_DIR, "debug_swarm_test.py")],
        output="screen",
    )


def generate_launch_description():
    actions = []

    # t=0s: drone 0 (leader) — starts Gazebo
    actions.append(px4_instance(0, 127.0, 52.67, 1.4))

    # t=5s: drones 1,2,3 — attach to running Gazebo
    follower_positions = [
        (1, 129.92, 52.852, 1.4),
        (2, 129.08, 54.095, 1.4),
        (3, 128.24, 55.339, 1.4),
    ]
    for idx, x, y, z in follower_positions:
        actions.append(TimerAction(
            period=5.0,
            actions=[px4_instance(idx, x, y, z)],
        ))

    # t=25s: single-process debug orchestrator (waits another 15s for EKF itself)
    actions.append(TimerAction(
        period=25.0,
        actions=[debug_test_process()],
    ))

    return LaunchDescription(actions)
