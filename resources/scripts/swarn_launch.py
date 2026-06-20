import math
import os
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
LED_PLUGIN_BUILD_DIR = os.path.join(os.path.dirname(SCRIPTS_DIR), "plugins", "led_controller", "build")

# Read the swarm size and train formation from the central config so the number
# of drones SPAWNED here stays in sync with the number the control script
# CONNECTS to (config.yaml runtime.follower_count). Without this they drift:
# bumping follower_count alone makes the controller wait on drones that were
# never launched.
_REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPTS_DIR))
sys.path.insert(0, _REPO_ROOT)
from drone_sdk.config import CONFIG

FOLLOWER_COUNT = CONFIG.runtime.follower_count
TRAIN_YAW_RAD = CONFIG.formation.train_yaw_rad
SPAWN_Z_M = CONFIG.formation.spawn_z_m
TRAIN_SPACING_M = CONFIG.formation.train_spacing_m

def leader_instanse(x, y, z, yaw=TRAIN_YAW_RAD):
        cmd = f"""
            export DISPLAY=:0
            export PX4_DIR="${{PX4_DIR:-$HOME/PX4-Autopilot}}"
            export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
            export GZ_RENDER_ENGINE=ogre2
            export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz:$PX4_DIR/Tools/simulation/gz/worlds:$PX4_DIR/Tools/simulation/gz/models:${{GZ_SIM_RESOURCE_PATH:-}}"
            export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_DIR/Tools/simulation/gz/plugins/led_controller/build:{LED_PLUGIN_BUILD_DIR}:$PX4_DIR/build/px4_sitl_default/build_gazebo:${{GZ_SIM_SYSTEM_PLUGIN_PATH:-}}"
            cd "$PX4_DIR" &&
            PX4_SYS_AUTOSTART=4010 \
            PX4_SIM_MODEL=gz_x500_mono_cam \
            PX4_GZ_WORLD=baylands_custom \
            PX4_GZ_MODEL_POSE="{x},{y},{z},0,0,{yaw}" \
            ./build/px4_sitl_default/bin/px4 -i 0
            """
        return ExecuteProcess(
                cmd=["bash", "-c", cmd],
                output="screen"
        )

def follower_instanse(i, x, y, z, yaw=TRAIN_YAW_RAD):
        cmd = f"""
            export DISPLAY=:0
            export PX4_DIR="${{PX4_DIR:-$HOME/PX4-Autopilot}}"
            export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
            export GZ_RENDER_ENGINE=ogre2
            export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz:$PX4_DIR/Tools/simulation/gz/worlds:$PX4_DIR/Tools/simulation/gz/models:${{GZ_SIM_RESOURCE_PATH:-}}"
            export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_DIR/Tools/simulation/gz/plugins/led_controller/build:{LED_PLUGIN_BUILD_DIR}:$PX4_DIR/build/px4_sitl_default/build_gazebo:${{GZ_SIM_SYSTEM_PLUGIN_PATH:-}}"
            cd "$PX4_DIR" &&
            PX4_SYS_AUTOSTART=4010 \
            PX4_SIM_MODEL=gz_x500_mono_cam \
            PX4_GZ_WORLD=baylands_custom \
            PX4_GZ_MODEL_POSE="{x},{y},{z},0,0,{yaw}" \
            ./build/px4_sitl_default/bin/px4 -i {i}
            """
        return ExecuteProcess(
                cmd=["bash", "-c", cmd],
                output="screen"
        )

def generate_launch_description():
        actions = []
        # Train formation:
        # - yaw is the train direction; all drones face the same way so cameras look forward.
        # - followers are spawned behind the previous drone along the opposite yaw vector.
        # - 2.0 m spacing keeps each follower aimed at the previous drone's rear LED markers.
        # - poses are always x,y,z,0,0,yaw to keep drones upright.
        leader_x = 127.0
        leader_y = 52.67
        behind_dx = -TRAIN_SPACING_M * math.cos(TRAIN_YAW_RAD)
        behind_dy = -TRAIN_SPACING_M * math.sin(TRAIN_YAW_RAD)

        # Leader (Starts Gazebo)
        actions.append(
                leader_instanse(
                        leader_x, 
                        leader_y, 
                        SPAWN_Z_M,
                        TRAIN_YAW_RAD
                    )
        )
        # Followers — generated from FOLLOWER_COUNT (config.yaml) so the swarm
        # size is set in one place. Each follower sits one TRAIN_SPACING_M step
        # further behind the leader along the train yaw.
        followers = [
                (
                        idx,
                        leader_x + idx * behind_dx,
                        leader_y + idx * behind_dy,
                        SPAWN_Z_M,
                )
                for idx in range(1, FOLLOWER_COUNT + 1)
        ]

        #Delay followers so Gazebo is ready
        for idx, x, y, z in followers:
                actions.append(
                        TimerAction(
                                period=5.0,
                                actions=[
                                        follower_instanse(
                                                idx, x, y, z, TRAIN_YAW_RAD
                                                )
                                        ]
                        )
                )
        return LaunchDescription(actions)
