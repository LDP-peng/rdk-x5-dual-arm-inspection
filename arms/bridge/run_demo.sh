#!/bin/bash
# 板A 一键启动（终极版）：底盘 + 雷达 + 图传，不启动 Nav2/AMCL。
#   bash ~/run_demo.sh         启动
#   bash ~/run_demo.sh status  只看状态
#   bash ~/run_demo.sh stop    全部停止

source /opt/tros/humble/setup.bash 2>/dev/null
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
set -u

have_topic() {
  timeout 5 ros2 topic echo "$1" --once --qos-reliability best_effort >/dev/null 2>&1
}

rate_of() {
  timeout 7 stdbuf -oL ros2 topic hz "$1" 2>/dev/null | grep -m1 'average rate' || echo '  无数据'
}

stop_all() {
  pkill -f '[o]dom_mission.py' 2>/dev/null
  pkill -f mjpeg_server.py 2>/dev/null
  pkill -f 'robot.launch.py' 2>/dev/null
  pkill -f nav2.launch.py 2>/dev/null
  pkill -f controller_server 2>/dev/null
  pkill -f planner_server 2>/dev/null
  pkill -f bt_navigator 2>/dev/null
  pkill -f amcl 2>/dev/null
  pkill -f map_server 2>/dev/null
  pkill -f static_transform_publisher 2>/dev/null
  pkill -f c30d_driver 2>/dev/null
  pkill -f lslidar_driver_node 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null
  sleep 1
}

status() {
  echo "===== 节点 ====="
  ros2 node list 2>/dev/null | sort
  echo
  echo "===== /odom ====="
  rate_of /odom
  echo "===== /scan ====="
  rate_of /scan
  echo "===== 图传 ====="
  if ss -ltn 2>/dev/null | grep -q ':8080'; then
    echo "  8080 在监听"
  else
    echo "  8080 没有监听（看 /tmp/cam.log）"
  fi
  if ss -ltn 2>/dev/null | grep -q ':8081'; then
    echo "  8081 在监听"
  else
    echo "  8081 没有监听（看 /tmp/cam2.log）"
  fi
  echo
  echo "单画面: http://$(hostname -I | awk '{print $1}'):8080/"
  echo "双画面: http://$(hostname -I | awk '{print $1}'):8080/dual"
}

case "${1:-start}" in
  status)
    status
    exit 0
    ;;
  stop)
    stop_all
    echo "已全部停止"
    exit 0
    ;;
  start)
    ;;
  *)
    echo "用法: bash ~/run_demo.sh [start|status|stop]"
    exit 1
    ;;
esac

if [ ! -f "$HOME/robot_ws/install/setup.bash" ]; then
  echo "robot_ws 没编译或路径不对"
  exit 1
fi

stop_all

echo "[1/2] 启动底盘 + 雷达"
nohup ros2 launch robot_bringup robot.launch.py use_camera:=false \
  >/tmp/bringup.log 2>&1 &

ok_odom=0
ok_scan=0
for i in $(seq 1 25); do
  if [ "$ok_odom" = 0 ] && have_topic /odom; then ok_odom=1; fi
  if [ "$ok_scan" = 0 ] && have_topic /scan; then ok_scan=1; fi
  if [ "$ok_odom" = 1 ] && [ "$ok_scan" = 1 ]; then break; fi
  sleep 1
done

if [ "$ok_odom" = 0 ]; then
  echo "  /odom 没起来，看 /tmp/bringup.log"
  tail -20 /tmp/bringup.log
  exit 1
fi
echo "  /odom OK"

if [ "$ok_scan" = 0 ]; then
  echo "  /scan 没起来，看 /tmp/bringup.log（雷达串口/符号链接）"
  tail -20 /tmp/bringup.log
  exit 1
fi
echo "  /scan OK"

if [ "${START_FIRST_CAM:-1}" = "1" ]; then
  echo "[2/2] 启动第一路图传 8080"
  nohup python3 "$HOME/mjpeg_server.py" --device /dev/video0 --port 8080 \
    --width 640 --height 480 --fps 15 --quality 50 \
    >/tmp/cam.log 2>&1 &
  sleep 3

  if ! ss -ltn 2>/dev/null | grep -q ':8080'; then
    echo "  图传没起来，看 /tmp/cam.log"
    tail -20 /tmp/cam.log
  else
    echo "  图传 OK"
  fi
else
  echo "[2/2] 跳过 8080，保留 /dev/video0 给机械臂相机"
fi

if [ "${START_SECOND_CAM:-1}" = "1" ] && [ -e /dev/video2 ]; then
  echo "[2/2] 启动第二路图传 8081"
  nohup python3 "$HOME/mjpeg_server.py" --device /dev/video2 --port 8081 \
    --width 640 --height 480 --fps 10 --quality 40 \
    >/tmp/cam2.log 2>&1 &
  sleep 1
  if ss -ltn 2>/dev/null | grep -q ':8081'; then
    echo "  第二路图传 OK"
  else
    echo "  第二路图传没起来，看 /tmp/cam2.log"
  fi
fi

echo
status
echo
echo "下一步: python3 ~/odom_mission.py --ros-args -p target_x:=1.0 -p target_y:=0.0"
