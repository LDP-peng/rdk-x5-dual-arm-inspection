#!/bin/bash
# 板A：主从演示一键启动。
#   首选：GRIP=1 bash ~/run_full_demo.sh
#   夹爪未调好：GRIP=0 bash ~/run_full_demo.sh
#   距离调整：DISTANCE=1.5 SPEED=0.08 GRIP=1 bash ~/run_full_demo.sh

source /opt/tros/humble/setup.bash 2>/dev/null
source /opt/ros/humble/setup.bash
source "$HOME/robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
set -u

DISTANCE="${DISTANCE:-2.0}"
SPEED="${SPEED:-0.08}"
GRIP="${GRIP:-1}"
START_FIRST_CAM="${START_FIRST_CAM:-0}"
START_SECOND_CAM="${START_SECOND_CAM:-1}"
SIGNS='[1.0,-1.0,1.0,1.0,1.0]'
SLAVE_LOG=/tmp/slave_joint.log
SLAVE_PID=

cleanup() {
  if [ -n "$SLAVE_PID" ] && kill -0 "$SLAVE_PID" 2>/dev/null; then
    kill "$SLAVE_PID" 2>/dev/null
    wait "$SLAVE_PID" 2>/dev/null
  fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

if [ ! -f "$HOME/slave_joint_node.py" ]; then
  echo "缺少 ~/slave_joint_node.py，先把 arms/bridge/slave_joint_node.py 拷到板A"
  exit 1
fi
if [ ! -f "$HOME/run_open_demo.sh" ]; then
  echo "缺少 ~/run_open_demo.sh，先把 arms/bridge/run_open_demo.sh 拷到板A"
  exit 1
fi

pkill -f slave_joint_node.py 2>/dev/null
sleep 1

echo "[1/2] 启动从臂映射（scale=0.3，第二关节反向）"
if [ "$GRIP" = "1" ]; then
  python3 "$HOME/slave_joint_node.py" --ros-args \
    -p scale:=0.3 -p verbose:=true \
    -p grip_index:=6 -p grip_sign:=-1.0 -p grip_span:=500 \
    -p signs:="$SIGNS" >"$SLAVE_LOG" 2>&1 &
else
  python3 "$HOME/slave_joint_node.py" --ros-args \
    -p scale:=0.3 -p verbose:=true -p grip_index:=99 \
    -p signs:="$SIGNS" >"$SLAVE_LOG" 2>&1 &
fi
SLAVE_PID=$!
sleep 3

if kill -0 "$SLAVE_PID" 2>/dev/null; then
  echo "  从臂节点 OK（pid $SLAVE_PID），日志 $SLAVE_LOG"
else
  echo "  从臂节点启动失败，先不要演示主从。日志："
  tail -40 "$SLAVE_LOG"
  echo "  检查扩展板 12V、I2C 0x15、SDA/SCL/GND。"
  echo "  现在继续只演示底盘；修好后再重启本脚本。"
fi

echo "[2/2] 启动底盘 + 雷达 + 前进任务"
DISTANCE="$DISTANCE" SPEED="$SPEED" \
  START_FIRST_CAM="$START_FIRST_CAM" START_SECOND_CAM="$START_SECOND_CAM" \
  bash "$HOME/run_open_demo.sh"
