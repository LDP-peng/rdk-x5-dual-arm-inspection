#!/bin/bash
# 板A：保留 Nav2 的一键启动（底盘 + 雷达 + map_server + AMCL + 规划/控制）
#   bash ~/run_nav2_demo.sh
#   INIT_X=0.0 INIT_Y=0.0 INIT_YAW=0.0 bash ~/run_nav2_demo.sh
#   MAP=$HOME/robot_ws/maps/nav_map.yaml bash ~/run_nav2_demo.sh

source /opt/tros/humble/setup.bash 2>/dev/null
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
set -u

MAP="${MAP:-$HOME/robot_ws/maps/nav_map.yaml}"
INIT_X="${INIT_X:-0.0}"
INIT_Y="${INIT_Y:-0.0}"
INIT_YAW="${INIT_YAW:-0.0}"

stop_all() {
  pkill -f odom_mission.py 2>/dev/null
  pkill -f mjpeg_server.py 2>/dev/null
  pkill -f 'robot.launch.py' 2>/dev/null
  pkill -f nav2.launch.py 2>/dev/null
  pkill -f controller_server 2>/dev/null
  pkill -f planner_server 2>/dev/null
  pkill -f bt_navigator 2>/dev/null
  pkill -f amcl 2>/dev/null
  pkill -f map_server 2>/dev/null
  pkill -f static_transform_publisher 2>/dev/null
  pkill -f component_container_isolated 2>/dev/null
  pkill -f nav2_container 2>/dev/null
  pkill -f lifecycle_manager 2>/dev/null
  pkill -f velocity_smoother 2>/dev/null
  pkill -f behavior_server 2>/dev/null
  pkill -f waypoint_follower 2>/dev/null
  pkill -f smoother_server 2>/dev/null
  pkill -f c30d_driver 2>/dev/null
  pkill -f lslidar_driver_node 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null
  sleep 2
  pkill -9 -f component_container_isolated 2>/dev/null
  pkill -9 -f nav2_container 2>/dev/null
  sleep 1
}

state_of() {
  timeout -k 2 2 ros2 lifecycle get "$1" 2>/dev/null | awk '{print $1}'
}

if [ ! -f "$MAP" ]; then
  echo "地图不存在: $MAP"
  exit 1
fi

stop_all

echo "[1/3] 底盘 + 雷达"
nohup ros2 launch robot_bringup robot.launch.py use_camera:=false \
  >/tmp/bringup.log 2>&1 &
sleep 6

if ! timeout -k 2 5 ros2 topic echo /odom --once --qos-reliability best_effort >/dev/null 2>&1; then
  echo "  /odom 没起来"; tail -20 /tmp/bringup.log; exit 1
fi
if ! timeout -k 2 5 ros2 topic echo /scan --once --qos-reliability best_effort >/dev/null 2>&1; then
  echo "  /scan 没起来"; tail -20 /tmp/bringup.log; exit 1
fi
echo "  /odom /scan OK"

echo "[2/3] Nav2"
nohup ros2 launch robot_navigation nav2.launch.py map:="$MAP" \
  >/tmp/nav2.log 2>&1 &

echo "  等待 Nav2 启动（15 秒）..."
sleep 15

if ! timeout -k 2 5 ros2 node list 2>/dev/null | grep -q '/amcl'; then
  echo "  没看到 /amcl，Nav2 可能没起来"
  tail -40 /tmp/nav2.log
  exit 1
fi
echo "  Nav2 已启动（/amcl 在）"

echo "[3/3] 发初始位姿 ($INIT_X, $INIT_Y, ${INIT_YAW} rad)"
QZ=$(python3 -c "import math; print(math.sin($INIT_YAW/2.0))")
QW=$(python3 -c "import math; print(math.cos($INIT_YAW/2.0))")
# DDS 发现需要时间，--once 容易在匹配前就发丢；连发 5 次
timeout -k 2 6 ros2 topic pub -r 2 --times 5 /initialpose \
geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: $INIT_X, y: $INIT_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}" \
  >/tmp/initpose.log 2>&1 || true

echo "  等 AMCL 建立 map->odom ..."
amcl_ok=0
for i in $(seq 1 6); do
  if timeout -k 2 4 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q 'Translation'; then
    amcl_ok=1
    break
  fi
  sleep 1
done

if [ "$amcl_ok" = "1" ]; then
  echo "  AMCL OK：map->odom 已建立"
  timeout -k 2 4 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -8
else
  echo "  AMCL 没有建立 map->odom"
  if [ "${FALLBACK_STATIC:-1}" = "1" ]; then
    echo "  切静态 map->odom：Nav2 的规划/控制/代价地图保留，AMCL 退出定位"
    timeout -k 2 3 ros2 lifecycle set /amcl deactivate >/dev/null 2>&1 || true
    pkill -x amcl 2>/dev/null
    sleep 1
    nohup ros2 run tf2_ros static_transform_publisher \
      "$INIT_X" "$INIT_Y" 0 "$INIT_YAW" 0 0 map odom \
      >/tmp/static_tf.log 2>&1 &
    sleep 2
    echo "  map->base_link:"
    timeout -k 2 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -8
  else
    echo "  自动诊断："
    bash "$HOME/nav2_probe.sh" || true
  fi
fi

nohup python3 "$HOME/mjpeg_server.py" --device /dev/video0 --port 8080 \
  >/tmp/cam.log 2>&1 &
sleep 2

echo
echo "画面: http://$(hostname -I | awk '{print $1}'):8080/"
echo "导航目标:"
echo 'ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"'
