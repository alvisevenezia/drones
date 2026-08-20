#!/usr/bin/env bash
# =====================================================================
# Pont Gazebo -> ROS 2
# RGB + Depth + CameraInfo + IMU + ToF(2304) + CLOCK
# =====================================================================

set -o pipefail

source /opt/ros/jazzy/setup.bash

BASE="/world/tugbot_depot/model/x500_depth_0/link/camera_link/sensor"

RGB="${BASE}/IMX214/image"
CAMINFO="${BASE}/IMX214/camera_info"
IMU="${BASE}/camera_imu/imu"

echo "==> Pont gz -> ROS 2"
echo "    RGB (mono): /camera/rgb/image_raw"
echo "    CameraInfo: /camera/rgb/camera_info"
echo "    IMU       : /camera/imu"
echo "    ToF L9CX  : /tof/points   (source de profondeur)"
echo "    Clock     : /clock"
echo "    (depth /camera/depth/image_raw = PROJECTION du ToF, pas la camera)"
echo ""

ros2 run ros_gz_bridge parameter_bridge \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  "${RGB}@sensor_msgs/msg/Image[gz.msgs.Image" \
  "${CAMINFO}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  "${IMU}@sensor_msgs/msg/Imu[gz.msgs.IMU" \
  "/tof_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "/tof_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "/tof_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "/tof_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  "/tof_down/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" \
  --ros-args \
  -r "${RGB}:=/camera/rgb/image_raw" \
  -r "${CAMINFO}:=/camera/rgb/camera_info" \
  -r "${IMU}:=/camera/imu"