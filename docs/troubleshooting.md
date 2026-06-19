# Troubleshooting

## Gazebo Does Not Render Correctly in WSL2

Set WSL display variables if needed:

```bash
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export PULSE_SERVER=/mnt/wslg/PulseServer
```

For hybrid GPU systems:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

The repository setup script currently prefers OGRE2/OpenGL:

```bash
export GZ_RENDER_ENGINE=ogre2
```

Older notes mention `GZ_RENDER_ENGINE=vulkan` as a possible workaround, but the current `px4_gz_setup.sh` says Vulkan falls back to `llvmpipe` in the tested WSL setup.

## `project_setup.sh` Fails

The script expects:

```text
~/PX4-Autopilot/Tools/simulation/gz
```

It also expects PX4 model/world folders to exist. Build PX4 SITL once first:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Then rerun:

```bash
./project_setup.sh
```

## ROS 2 Commands Are Missing

Source ROS 2 in each new terminal:

```bash
source /opt/ros/humble/setup.bash
```

If your system uses a different ROS 2 distribution:

```bash
source /opt/ros/<distro>/setup.bash
```

## MAVSDK Import Fails

Install MAVSDK into the same Python interpreter used to run scripts:

```bash
python3 -m pip install mavsdk
python3 -c "import mavsdk; print(mavsdk.__file__)"
```

If using a virtual environment, activate it before both installation and script execution.

## Camera Frames Are Always `None`

Check:

- Gazebo/PX4 swarm is running.
- ROS 2 is sourced.
- `ros_gz_image` is installed and available.
- `drone.start_camera()` has been called.
- Your loop calls `drone.spin()` before `drone.camera_frame()`.
- The camera topic matches the drone id:

```text
/world/baylands_custom/model/x500_mono_cam_{id}/link/mono_cam/base_link/sensor/camera_sensor/image
```

## LED Commands Do Not Change the Model

Check:

- `project_setup.sh` copied `resources/plugins/led_controller` into PX4.
- `px4_gz_setup.sh` was sourced before launching the world.
- `GZ_SIM_SYSTEM_PLUGIN_PATH` includes:

```text
~/PX4-Autopilot/Tools/simulation/gz/plugins/led_controller/build
```

- `ros_gz_bridge` is installed.
- Commands are sent to:

```text
/model/x500_mono_cam_{id}/led_cmd
```

## Offboard Start Fails

PX4 requires setpoints before offboard mode starts. Use the SDK method:

```python
await drone.start_offboard()
```

It sends repeated initial position setpoints before calling MAVSDK offboard start.

## Simulation Keeps Running After Ctrl+C

Stop from the PX4 launch terminal with `Ctrl+C`. If a PX4 process remains:

```bash
pkill -9 px4
```

