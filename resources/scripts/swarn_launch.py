import math
import os

from launch import LaunchDescription    
from launch.actions import ExecuteProcess, TimerAction

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
LED_PLUGIN_BUILD_DIR = os.path.join(os.path.dirname(SCRIPTS_DIR), "plugins", "led_controller", "build")
TRAIN_YAW_RAD = 3.7346
SPAWN_Z_M = 1.4
TRAIN_SPACING_M = 2.0

def leader_instanse(x, y, z, yaw=3.7346):
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

def follower_instanse(i, x, y, z, yaw=3.7346):
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
        # Followers
        followers = [
                (1, leader_x + behind_dx, leader_y + behind_dy, SPAWN_Z_M),
                (2, leader_x + 2 * behind_dx, leader_y + 2 * behind_dy, SPAWN_Z_M),
                (3, leader_x + 3 * behind_dx, leader_y + 3 * behind_dy, SPAWN_Z_M)
        ]

        #Delay followers so Gazebo is ready
        for idx, x, y, z in followers:
                actions.append(
                        TimerAction(
                                period=5.0,
                                actions=[
                                        follower_instanse(
                                                idx, x, y, z
                                                )
                                        ]
                        )
                )
        return LaunchDescription(actions)
