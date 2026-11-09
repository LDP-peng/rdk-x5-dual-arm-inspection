#!/usr/bin/env bash
set -u

source /opt/tros/humble/setup.bash

echo "== OS =="
cat /etc/version 2>/dev/null || true
rdkos_info 2>/dev/null || true

echo "== ROS =="
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
ros2 topic list 2>/dev/null || true

echo "== Required packages =="
for pkg in \
  turn_on_wheeltec_robot \
  wheeltec_nav2 \
  lslidar_driver \
  lslidar_msgs \
  usb_cam \
  robot_localization \
  nav2_bringup \
  slam_toolbox; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "OK   $pkg"
  else
    echo "MISS $pkg"
  fi
done

echo "== Devices =="
ls -l /dev/wheeltec_* 2>/dev/null || echo "no /dev/wheeltec_* aliases"

echo "== USB =="
lsusb 2>/dev/null || true

echo "== Video devices =="
ls -l /dev/video* 2>/dev/null || echo "no /dev/video*"
v4l2-ctl --list-devices 2>/dev/null || true
v4l2-ctl --device=/dev/video0 --list-formats-ext 2>/dev/null || true

echo "== Topics =="
for topic in /odom /scan /image_raw /vision/fire_smoke/detections /mission/state; do
  echo "--- $topic"
  timeout 2 ros2 topic hz "$topic" 2>/dev/null || true
done
