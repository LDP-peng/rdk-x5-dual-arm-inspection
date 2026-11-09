#!/bin/bash
# 板A 一键启动：底盘+雷达 -> 静态TF -> Nav2 -> 图传
# 用法：bash ~/run_robot.sh

source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
set -u

pkill -f "robot.launch.py" 2>/dev/null
pkill -f "nav2.launch.py" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
pkill -f "mjpeg_server.py" 2>/dev/null
sleep 1

MAP="$HOME/robot_ws/maps/nav_map.yaml"
if [ ! -f "$MAP" ]; then
  echo "地图不存在: $MAP"
  exit 1
fi

echo "[1/4] bringup"
nohup ros2 launch robot_bringup robot.launch.py use_camera:=false \
  >/tmp/bringup.log 2>&1 &
sleep 6

echo "[2/4] static TF map->odom"
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom \
  >/tmp/tf.log 2>&1 &
sleep 1

echo "[3/4] nav2"
nohup ros2 launch robot_navigation nav2.launch.py map:="$MAP" \
  >/tmp/nav2.log 2>&1 &
sleep 1

echo "[4/4] camera"
nohup python3 "$HOME/mjpeg_server.py" --device /dev/video0 --port 8080 \
  >/tmp/cam.log 2>&1 &

sleep 8

echo
echo "===== 健康检查 ====="
echo "-- nodes --"
ros2 node list

echo "-- /scan --"
timeout 4 ros2 topic hz /scan | tail -2

echo "-- /map --"
timeout 5 ros2 topic echo /map --once --field info.resolution

echo "-- TF map->base_link --"
timeout 4 ros2 run tf2_ros tf2_echo map base_link | head -8

echo
echo "画面: http://$(hostname -I | awk '{print $1}'):8080/"
echo "日志: /tmp/bringup.log /tmp/nav2.log /tmp/cam.log"