#!/bin/bash
# 板A：AMCL / Nav2 全链路诊断。先跑 run_nav2_demo.sh，再跑这个。
#   bash ~/nav2_probe.sh
# 把整段输出发回来即可。

source /opt/tros/humble/setup.bash 2>/dev/null
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
set -u

# 诊断时只允许 AMCL 提供 map->odom，先清掉静态 TF 和上一轮残留
pkill -f static_transform_publisher 2>/dev/null
sleep 2

MAP="${MAP:-$HOME/robot_ws/maps/nav_map.yaml}"
INIT_X="${INIT_X:-0.0}"
INIT_Y="${INIT_Y:-0.0}"
INIT_YAW="${INIT_YAW:-0.0}"

echo "===== 0. 系统时间 ====="
date '+%Y-%m-%d %H:%M:%S.%N %Z'
echo

echo "===== 1. 节点 ====="
ros2 node list 2>/dev/null | sort
echo

echo "===== 2. 生命周期 ====="
for n in /map_server /amcl /planner_server /controller_server /bt_navigator; do
  printf "%-22s" "$n"
  timeout -k 2 3 ros2 lifecycle get "$n" 2>&1 | head -1
done
echo

echo "===== 3. 关键话题 ====="
ros2 topic list -t 2>/dev/null | grep -E '^/(scan|map|amcl_pose|initialpose|odom|cmd_vel|particlecloud)' || echo none
echo

echo "===== 4. /scan ====="
ros2 topic info /scan -v 2>&1 | sed -n '1,30p'
echo "-- hz --"
timeout -k 2 6 stdbuf -oL ros2 topic hz /scan 2>/dev/null | grep -m1 'average rate' || echo none
echo "-- stamp --"
timeout -k 2 4 ros2 topic echo /scan --once --field header.stamp --qos-reliability best_effort 2>&1 | head -4
echo

echo "===== 5. /odom ====="
ros2 topic info /odom -v 2>&1 | sed -n '1,30p'
echo "-- hz --"
timeout -k 2 6 stdbuf -oL ros2 topic hz /odom 2>/dev/null | grep -m1 'average rate' || echo none
echo "-- stamp --"
timeout -k 2 4 ros2 topic echo /odom --once --field header.stamp --qos-reliability best_effort 2>&1 | head -4
echo

echo "===== 6. /map ====="
ros2 topic info /map -v 2>&1 | sed -n '1,30p'
echo "-- info --"
timeout -k 2 6 ros2 topic echo /map --once --field info --qos-durability transient_local 2>&1 | head -20
echo "-- yaml --"
echo "$MAP"
[ -f "$MAP" ] && cat "$MAP"
echo

echo "===== 7. TF 链 ====="
for pair in "odom base_footprint" "base_footprint base_link" "base_link laser_link"; do
  set -- $pair
  echo "-- $1 -> $2"
  timeout -k 2 4 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | grep -m3 -E 'Translation|Rotation|Invalid|does not exist|Exception' || echo none
done
echo

echo "===== 8. AMCL 实际参数 ====="
for p in use_sim_time base_frame_id odom_frame_id global_frame_id scan_topic map_topic tf_broadcast set_initial_pose always_reset_initial_pose robot_model_type laser_model_type max_particles min_particles update_min_d update_min_a transform_tolerance; do
  printf "%-26s" "$p"
  timeout -k 2 3 ros2 param get /amcl "$p" 2>&1 | tail -1
done
echo

echo "===== 9. AMCL 订阅/发布 ====="
timeout -k 2 4 ros2 node info /amcl 2>&1 | sed -n '/Subscribers:/,/Service Servers:/p'
echo

echo "===== 10. 参数文件里的 amcl 段 ====="
PREFIX=$(ros2 pkg prefix wheeltec_nav2 2>/dev/null)
if [ -n "${PREFIX:-}" ]; then
  PARAM="$PREFIX/share/wheeltec_nav2/param/wheeltec_param/param_mini_diff.yaml"
  echo "$PARAM"
  [ -f "$PARAM" ] && sed -n '/^amcl:/,/^[a-zA-Z]/p' "$PARAM"
else
  echo "wheeltec_nav2 没找到"
fi
echo

echo "===== 11. 发初始位姿 ====="
QZ=$(python3 -c "import math; print(math.sin($INIT_YAW/2.0))")
QW=$(python3 -c "import math; print(math.cos($INIT_YAW/2.0))")
timeout -k 2 6 ros2 topic pub -r 2 --times 5 /initialpose \
geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: map}, pose: {pose: {position: {x: $INIT_X, y: $INIT_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}" \
  2>&1 | tail -3 || true
sleep 3
echo

echo "===== 12. /amcl_pose ====="
timeout -k 2 8 ros2 topic echo /amcl_pose --once --qos-reliability best_effort 2>&1 | head -30 || echo none
echo

echo "===== 13. map -> odom ====="
timeout -k 2 5 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m5 -E 'Translation|Rotation|Invalid|does not exist|Exception' || echo none
echo

echo "===== 14. Nav2 日志尾部 ====="
tail -80 /tmp/nav2.log 2>/dev/null || echo "没有 /tmp/nav2.log"
