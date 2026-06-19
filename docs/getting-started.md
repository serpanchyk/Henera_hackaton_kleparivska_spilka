# Getting Started

## Tested Baseline

The existing setup notes were tested on:

- Windows 11
- WSL2
- Ubuntu 22.04
- Gazebo Harmonic
- PX4 Autopilot `v1.15.4`
- ROS 2 Humble

Native Ubuntu 22.04 can also be used. If you use another ROS 2 version, replace `humble` in commands with the installed distribution name.

## Install Gazebo Harmonic

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic
```

Test:

```bash
gz sim
```

For WSL graphics issues, see [Troubleshooting](troubleshooting.md).

## Install PX4

Clone PX4 `v1.15.4`:

```bash
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.15.4
cd ~/PX4-Autopilot
git fetch --all --tags
git checkout v1.15.4
git submodule sync --recursive
git submodule update --init --recursive
```

Build and test the default Gazebo model once:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

If Python build dependencies are missing:

```bash
sudo apt install python3-pip
pip install --user empy==3.3.4 pyros-genmsg setuptools
pip3 install --user future symforce kconfiglib jinja2 jsonschema
```

## Install ROS 2 Humble

Set locale:

```bash
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Enable Universe and install ROS 2:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

Source ROS 2 in every terminal:

```bash
source /opt/ros/humble/setup.bash
```

Optional persistent setup:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Python Dependencies

Install MAVSDK into the same Python interpreter used to run examples:

```bash
python3 -m pip install mavsdk
python3 -c "import mavsdk; print(mavsdk.__file__)"
```

The SDK also expects ROS 2 packages for `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`, `ros_gz_image`, and `ros_gz_bridge`, plus OpenCV for the example camera windows.

## Copy Project Assets Into PX4

Run this only after PX4 exists and has been built at least once:

```bash
./project_setup.sh
```

The script copies:

- `resources/worlds/media`
- `resources/worlds/baylands_custom.config`
- `resources/worlds/baylands_custom.sdf`
- modified `x500_base` and `x500_mono_cam` model files
- `resources/plugins/led_controller`

Target location:

```text
~/PX4-Autopilot/Tools/simulation/gz
```

## Load Gazebo/PX4 Environment

Before launching the custom world:

```bash
cd ~/PX4-Autopilot
source ~/falcon_gaze/resources/scripts/px4_gz_setup.sh
```

Adjust the path if this repository is not located at `~/falcon_gaze`.

## Launch the Swarm

In a ROS-sourced terminal:

```bash
cd ~/PX4-Autopilot
ros2 launch ~/falcon_gaze/resources/scripts/swarn_launch.py
```

This starts PX4 and opens Gazebo with four drones: one leader and three followers.

To stop the simulation, press `Ctrl+C` in the PX4 launch terminal. If needed:

```bash
pkill -9 px4
```

## Run the Leader Mission Script

In another ROS-sourced terminal:

```bash
cd ~/falcon_gaze/resources/scripts
python3 mission_launch.py
```

This runs a simple straight leader mission using MAVSDK.

