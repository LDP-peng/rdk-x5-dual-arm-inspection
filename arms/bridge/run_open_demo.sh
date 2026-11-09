#!/bin/bash
# 板A：空旷场地一键演示。里程计直行 -> 停车遥操作 -> 回车返航。
#   bash ~/run_open_demo.sh
#   DISTANCE=2.0 SPEED=0.08 bash ~/run_open_demo.sh

source /opt/tros/humble/setup.bash 2>/dev/null
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
set -u

DISTANCE="${DISTANCE:-2.0}"
SPEED="${SPEED:-0.10}"

if [ ! -f "$HOME/robot_ws/install/setup.bash" ]; then
  echo "robot_ws 没编译或路径不对"
  exit 1
fi
if [ ! -f "$HOME/odom_mission.py" ]; then
  echo "缺少 ~/odom_mission.py，先把 arms/bridge/odom_mission.py 拷到板A"
  exit 1
fi

echo "目标：沿车头前进 ${DISTANCE} m，速度 ${SPEED} m/s"
echo "先确认车头正前方 2 m 内无人、无障碍"
read -r -p "按回车开始，Ctrl+C 随时停车..." _

bash "$HOME/run_demo.sh" || exit 1

python3 "$HOME/odom_mission.py" --ros-args \
  -p target_x:="$DISTANCE" \
  -p target_y:=0.0 \
  -p target_yaw:=0.0 \
  -p max_linear:="$SPEED" \
  -p front_stop:=0.45
