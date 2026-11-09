#!/bin/bash
# 双机械臂主从遥操作演示：先底盘避障，到点后再启动主从映射。
#
# 车端：
#   bash ~/run_demo_final.sh car
#   DISTANCE=1.5 SPEED=0.08 bash ~/run_demo_final.sh car
#
# 主臂板（必须等车到巡检点后再执行）：
#   bash ~/run_demo_final.sh main

set -u

MODE="${1:-car}"
export ROS_DOMAIN_ID=0

DISTANCE="${DISTANCE:-2.0}"
SPEED="${SPEED:-0.08}"
SCALE="${SCALE:-0.6}"
GRIP="${GRIP:-1}"
GRIP_GAIN="${GRIP_GAIN:-1.0}"
GRIP_SPAN="${GRIP_SPAN:-500}"
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

run_main_arm() {
  source /opt/ros/humble/setup.bash

  if [ ! -f "$HOME/master_drag_node.py" ] || \
     [ ! -f "$HOME/urdf_gravity.py" ]; then
    echo "缺少 ~/master_drag_node.py 或 ~/urdf_gravity.py"
    exit 1
  fi

  pkill -f master_joint_node.py 2>/dev/null || true
  pkill -f master_drag_node.py 2>/dev/null || true

  echo
  echo "即将启动主臂重力补偿。"
  echo "请先扶住主臂；Ctrl+C 退出时电机会卸力，主臂会下落。"
  read -r -p "扶稳后按回车启动..." _

  exec python3 "$HOME/master_drag_node.py"
}

start_slave() {
  if [ ! -f "$HOME/slave_joint_node.py" ]; then
    echo "缺少 ~/slave_joint_node.py，跳过从臂映射。"
    return 1
  fi

  pkill -f slave_joint_node.py 2>/dev/null || true
  sleep 1

  local args=(
    --ros-args
    -p "scale:=$SCALE"
    -p "verbose:=true"
    -p "signs:=$SIGNS"
  )
  if [ "$GRIP" = "1" ]; then
    args+=(
      -p "grip_index:=6"
      -p "grip_sign:=-1.0"
      -p "grip_gain:=$GRIP_GAIN"
      -p "grip_span:=$GRIP_SPAN"
    )
  else
    args+=(-p "grip_index:=99")
  fi

  python3 "$HOME/slave_joint_node.py" "${args[@]}" \
    >"$SLAVE_LOG" 2>&1 &
  SLAVE_PID=$!
  sleep 3

  if kill -0 "$SLAVE_PID" 2>/dev/null; then
    echo "从臂节点已待机（尚未收到主臂数据，不会动作）"
    return 0
  fi

  echo "从臂节点启动失败，底盘仍可继续演示。日志："
  tail -30 "$SLAVE_LOG"
  echo "检查扩展板 12V、I2C 0x15、SDA/SCL/GND。"
  return 1
}

run_car() {
  source /opt/tros/humble/setup.bash 2>/dev/null || true
  source /opt/ros/humble/setup.bash
  source "$HOME/robot_ws/install/setup.bash"

  if [ ! -f "$HOME/run_demo.sh" ] || [ ! -f "$HOME/odom_mission.py" ]; then
    echo "缺少 ~/run_demo.sh 或 ~/odom_mission.py"
    exit 1
  fi

  pkill -f visual_pick.py 2>/dev/null || true
  pkill -f official_pick.py 2>/dev/null || true
  pkill -f color_grab.py 2>/dev/null || true
  pkill -f arm_teach.py 2>/dev/null || true
  pkill -f odom_mission.py 2>/dev/null || true

  echo "[1/3] 启动底盘、雷达、图传"
  START_FIRST_CAM="$START_FIRST_CAM" START_SECOND_CAM="$START_SECOND_CAM" \
    bash "$HOME/run_demo.sh" || exit 1

  local car_ip
  car_ip=$(hostname -I | awk '{print $1}')
  echo
  echo "图传单画面：http://${car_ip}:8080/"
  echo "图传双画面：http://${car_ip}:8080/dual"
  echo "机械臂相机：http://${car_ip}:8081/"

  echo
  echo "[2/3] 启动从臂节点（待机，不动作）"
  start_slave || true

  echo
  echo "[3/3] 底盘避障巡检"
  echo "目标：前进 ${DISTANCE} m，速度 ${SPEED} m/s"
  echo "车到巡检点前不要启动主臂。"
  echo "车到点后，在主臂板执行：bash ~/run_demo_final.sh main"
  read -r -p "确认车头前方 2 m 内无人、无障碍，按回车开始..." _

  python3 "$HOME/odom_mission.py" --ros-args \
    -p "target_x:=$DISTANCE" \
    -p "target_y:=0.0" \
    -p "target_yaw:=0.0" \
    -p "max_linear:=$SPEED" \
    -p "front_stop:=0.45"
  local code=$?

  echo
  if [ "$code" -eq 0 ]; then
    echo "巡检演示完成。"
  else
    echo "巡检退出，代码 $code。"
  fi
  exit "$code"
}

case "$MODE" in
  car)
    run_car
    ;;
  main)
    run_main_arm
    ;;
  *)
    echo "用法：bash ~/run_demo_final.sh [car|main]"
    echo "  car  = 车端，先避障巡检"
    echo "  main = 主臂板，车到点后启动重力补偿"
    exit 1
    ;;
esac
