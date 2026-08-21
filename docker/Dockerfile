FROM nvidia/cuda:12.6.3-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Brussels
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

SHELL ["/bin/bash", "-c"]

# ============================================================
# Base Ubuntu
# ============================================================

RUN apt-get update && apt-get install -y \
    locales \
    tzdata \
    sudo \
    curl \
    wget \
    git \
    git-lfs \
    vim \
    nano \
    bash-completion \
    build-essential \
    cmake \
    ninja-build \
    ccache \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-argcomplete \
    software-properties-common \
    ca-certificates \
    gnupg \
    lsb-release \
    unzip \
    zip \
    rsync \
    net-tools \
    iproute2 \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*


RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ============================================================
# ROS 2 Jazzy
# ============================================================

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-base \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-vision-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    ros-dev-tools \
    ros-jazzy-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Gazebo Harmonic
# ============================================================

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    gz-harmonic \
    libgz-sim8-dev \
    libgz-transport13-dev \
    libgz-msgs10-dev \
    libgz-sensors8-dev \
    libgz-plugin2-dev \
    libgz-math7-dev \
    libgz-common6-dev \
    libgz-rendering8-dev \
    libgz-physics7-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# PX4 build dependencies
# ============================================================

RUN apt-get update && apt-get install -y \
    gcc-arm-none-eabi \
    g++-multilib \
    gcc-multilib \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    genromfs \
    kconfig-frontends \
    exiftool \
    astyle \
    clang \
    clang-format \
    protobuf-compiler \
    libprotobuf-dev \
    libeigen3-dev \
    libxml2-utils \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Micro XRCE-DDS Agent — communication PX4 <-> ROS 2
# ============================================================

RUN cd /tmp && \
    git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# ============================================================
# Python dependencies
# ============================================================

RUN python3 -m pip install --break-system-packages \
    empy \
    pyros-genmsg \
    kconfiglib \
    jsonschema \
    packaging \
    numpy \
    scipy \
    matplotlib \
    pyyaml \
    jinja2 \
    toml \
    setuptools

# ============================================================
# PX4 v1.18
# ============================================================

WORKDIR /workspace

RUN git clone \
    https://github.com/PX4/PX4-Autopilot.git \
    /workspace/PX4-Autopilot

WORKDIR /workspace/PX4-Autopilot

RUN git checkout caa62044d0 && \
    git submodule update --init --recursive

# Make sure submodules are present
RUN git submodule update --init --recursive

COPY worlds/tugbot_depot.sdf /workspace/PX4-Autopilot/Tools/simulation/gz/worlds/tugbot_depot.sdf

# ============================================================
# Build PX4
# ============================================================

WORKDIR /workspace/PX4-Autopilot

RUN make px4_sitl


# ============================================================
# PX4 environment
# ============================================================

ENV PX4_HOME=/workspace/PX4-Autopilot

RUN echo 'export PX4_HOME=/workspace/PX4-Autopilot' >> /root/.bashrc
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
RUN echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc

# ============================================================
# ROS 2 / Gazebo environment
# ============================================================

ENV GZ_VERSION=harmonic

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
RUN echo 'export GZ_VERSION=harmonic' >> /root/.bashrc

# ============================================================
# rosdep
# ============================================================

RUN rosdep init 2>/dev/null || true
RUN rosdep update


# ============================================================
# Useful diagnostics
# ============================================================

RUN echo '#!/bin/bash' > /usr/local/bin/check-sim && \
    echo 'echo "===== NVIDIA ====="' >> /usr/local/bin/check-sim && \
    echo 'nvidia-smi || true' >> /usr/local/bin/check-sim && \
    echo 'echo "===== Gazebo ====="' >> /usr/local/bin/check-sim && \
    echo 'gz sim --version' >> /usr/local/bin/check-sim && \
    echo 'echo "===== ROS 2 ====="' >> /usr/local/bin/check-sim && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /usr/local/bin/check-sim && \
    echo 'ros2 --version' >> /usr/local/bin/check-sim && \
    echo 'echo "===== PX4 ====="' >> /usr/local/bin/check-sim && \
    echo 'cd /workspace/PX4-Autopilot' >> /usr/local/bin/check-sim && \
    echo 'git describe --always --tags' >> /usr/local/bin/check-sim && \
    chmod +x /usr/local/bin/check-sim

# ============================================================
# Default shell
# ============================================================

WORKDIR /workspace

CMD ["/bin/bash"]