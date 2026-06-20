FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG PX4_VERSION=v1.15.4

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=Etc/UTC \
    ROS_DISTRO=humble \
    PX4_DIR=/root/PX4-Autopilot \
    FALCON_GAZE_DIR=/opt/falcon-gaze \
    PYTHONUNBUFFERED=1 \
    GZ_RENDER_ENGINE=ogre2

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        gnupg \
        lsb-release \
        locales \
        software-properties-common \
        tzdata \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ccache \
        cmake \
        ninja-build \
        pkg-config \
        python3-dev \
        python3-empy \
        python3-jinja2 \
        python3-numpy \
        python3-opencv \
        python3-packaging \
        python3-pip \
        python3-setuptools \
        python3-toml \
        python3-yaml \
        ros-humble-cv-bridge \
        ros-humble-desktop \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-image \
        ros-humble-vision-opencv \
        gz-harmonic \
        libgz-plugin2-dev \
        libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive --branch "${PX4_VERSION}" \
        https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}" \
    && cd "${PX4_DIR}" \
    && git submodule sync --recursive \
    && git submodule update --init --recursive \
    && make px4_sitl gz_x500

WORKDIR ${FALCON_GAZE_DIR}

COPY requirements.txt pyproject.toml ./
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir -r requirements.txt

COPY . ${FALCON_GAZE_DIR}

RUN chmod +x start_cv.sh project_setup.sh resources/scripts/px4_gz_setup.sh \
    && ./project_setup.sh \
    && cmake -S "${PX4_DIR}/Tools/simulation/gz/plugins/led_controller" \
             -B "${PX4_DIR}/Tools/simulation/gz/plugins/led_controller/build" \
    && cmake --build "${PX4_DIR}/Tools/simulation/gz/plugins/led_controller/build" --parallel

COPY docker/entrypoint.sh /usr/local/bin/falcon-gaze-entrypoint
RUN chmod +x /usr/local/bin/falcon-gaze-entrypoint

ENTRYPOINT ["/usr/local/bin/falcon-gaze-entrypoint"]
CMD ["bash", "start_cv.sh"]
