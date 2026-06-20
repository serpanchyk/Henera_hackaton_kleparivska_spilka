from launch.actions import ExecuteProcess


def px4_instance(instance_id: int, x: float, y: float, z: float, yaw: float):
    # Env vars are set explicitly so Gazebo gets them even in subprocess context.
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
    return ExecuteProcess(cmd=['bash', '-c', cmd], output='screen')
