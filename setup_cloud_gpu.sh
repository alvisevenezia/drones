#!/usr/bin/env bash
# =====================================================================
#  Bootstrap cloud GPU - ROS 2 Jazzy + PX4 + SLAM workspace
#  Usage: bash ./setup_cloud_gpu.sh
#
#  What it does:
#  - installs ROS 2 Jazzy desktop and build tools if needed
#  - installs the apt/rosdep dependencies declared by this repo
#  - clones PX4-Autopilot and Micro-XRCE-DDS-Agent if missing
#  - builds the Micro-XRCE-DDS Agent when source is available
#  - runs the existing privileged installer and then colcon builds the repo
# =====================================================================
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEBIAN_FRONTEND=noninteractive
export DEBIAN_FRONTEND

log() {
  printf '\n==> %s\n' "$*"
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing command: $1" >&2
    exit 1
  fi
}

sudo_apt_install() {
  sudo apt-get update
  sudo apt-get install -y "$@"
}

ensure_ros_repo() {
  if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
    return
  fi

  log "Installing ROS 2 ${ROS_DISTRO} apt repository"
  sudo_apt_install ca-certificates curl gnupg lsb-release software-properties-common
  sudo add-apt-repository universe -y >/dev/null
  sudo mkdir -p /usr/share/keyrings
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
}

ensure_external_repo() {
  local target_path="$1"
  local repo_url="$2"
  local clone_args="${3:-}"

  if [[ -d "$target_path/.git" ]]; then
    return
  fi

  log "Cloning $(basename "$target_path")"
  git clone ${clone_args} "$repo_url" "$target_path"
}

build_microxrce_agent() {
  local agent_path="$HOME/Micro-XRCE-DDS-Agent"
  local build_dir="$agent_path/build"
  local agent_bin="$build_dir/MicroXRCEAgent"

  if [[ -x "$agent_bin" ]]; then
    return
  fi

  if [[ ! -d "$agent_path/.git" ]]; then
    log "Micro-XRCE-DDS-Agent not cloned yet, skipping build"
    return
  fi

  log "Building Micro-XRCE-DDS Agent"
  cmake -S "$agent_path" -B "$build_dir" -DUCLIENT_PROFILE_UDP=ON -DUCLIENT_PROFILE_TCP=ON -DUCLIENT_PROFILE_DISCOVERY=ON
  cmake --build "$build_dir" -j"$(nproc)"
}

source_ros() {
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
}

main() {
  need_cmd sudo

  sudo_apt_install \
    ca-certificates \
    curl \
    gnupg \
    git \
    lsb-release \
    software-properties-common

  ensure_ros_repo
  sudo_apt_install \
    build-essential \
    cmake \
    ninja-build \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-rtabmap-ros \
    librtabmap-dev

  source_ros

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update

  ensure_external_repo "$HOME/PX4-Autopilot" https://github.com/PX4/PX4-Autopilot.git --recursive
  ensure_external_repo "$HOME/Micro-XRCE-DDS-Agent" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

  build_microxrce_agent

  log "Running privileged repo installer"
  bash "$REPO_ROOT/install_sudo.sh"

  log "Resolving workspace dependencies with rosdep"
  rosdep install --from-paths "$REPO_ROOT/dual_rviz_jazzy" "$REPO_ROOT/src/px4_keyboard_teleop" --ignore-src -r -y

  log "Building the workspace"
  colcon build --symlink-install --base-paths "$REPO_ROOT" "$REPO_ROOT/src"

  log "Setup complete"
  echo "Source this shell: source \"$REPO_ROOT/install/setup.bash\""
}

main "$@"