#!/usr/bin/env bash
# =====================================================================
#  Pont Unity <-> ROS 2  (ROS-TCP-Endpoint)
#  Usage :  bash ~/Documents/drones/run_unity_bridge.sh
#  A lancer dans un terminal dedie ; Unity s'y connecte sur 127.0.0.1:10000.
#  Ctrl+C pour arreter.
# =====================================================================
set -o pipefail

source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "==> Endpoint Unity <-> ROS 2 sur 0.0.0.0:10000"
echo "    Dans Unity : Robotics > ROS Settings > ROS IP = 127.0.0.1 , Port = 10000"
echo ""
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
