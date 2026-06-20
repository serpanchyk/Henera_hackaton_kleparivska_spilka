from launch.actions import ExecuteProcess
from pathlib import Path


LED_PLUGIN_BUILD_DIR = Path(__file__).resolve().parents[3] / "plugins" / "led_controller" / "build"


def px4_instance(instance_id: int, x: float, y: float, z: float, yaw: float):
    # Env vars are set explicitly so Gazebo gets them even in subprocess context.
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
        ./build/px4_sitl_default/bin/px4 -i {instance_id}
        """
    return ExecuteProcess(cmd=['bash', '-c', cmd], output='screen')
