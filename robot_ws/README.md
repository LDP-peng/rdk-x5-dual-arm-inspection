# robot_ws — RDK X5 差速巡检机器人

硬件配置已确定：

- 底盘：WHEELTEC L150Pro 差速底盘，C30D V2.0 STM32
- 雷达：N10P 串口版
- 相机：WHEELTEC C70 USB 相机
- 主控：RDK X5，Ubuntu 22.04 + TROS Humble

## 目录

```text
robot_ws/
├── src/
│   ├── robot_bringup/       # 一键启动底盘、雷达、相机、视觉
│   ├── robot_description/   # 差速 URDF、雷达/相机/IMU 外参
│   ├── robot_perception/    # 火/烟检测节点
│   ├── robot_mission/       # 巡检状态机
│   └── robot_navigation/    # Nav2 启动封装
├── maps/                    # 建图输出
├── models/                  # .pt / .onnx / .bin / 类别文件
├── bags/                    # rosbag2 调试数据
├── docs/                    # 标定与检查清单
└── tools/                   # 环境检查脚本
```

## 1. 依赖

在 RDK X5 上：

```bash
source /opt/tros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-usb-cam \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

`turn_on_wheeltec_robot`（含 `wheeltec_ekf.launch.py`）和 `wheeltec_nav2`
需要来自 WHEELTEC ROS2 镜像或完整源码包。若工作区没有完整
`wheeltec_ros2` 源码，请先向 WHEELTEC 获取并放入 `src/third_party/`，或
从已安装的 WHEELTEC 镜像中直接运行。N10P 的 `lslidar_driver` 和
`lslidar_msgs` 已从随附资料复制到 `src/third_party/`。

本机型对应 `wheeltec_params.yaml` 的关键配置：

```yaml
car_mode: mini_diff
lidar_type: ls_N10P_uart
imu_model: stm32
camera_model: usb
```

完整上板步骤见 `docs/WHEELTEC_ROS2_SETUP.md`。

## 2. 编译

```bash
cd ~/robot_ws
source /opt/tros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3. udev 固定串口

```bash
sudo cp src/robot_bringup/udev/99-wheeltec.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/wheeltec_*
```

应看到：

- `/dev/wheeltec_lidar`：N10P
- `/dev/wheeltec_controller`：C30D STM32

如果设备号不是 0001/0002，用 `lsusb -v` 或 `udevadm info` 查实际 serial 后修改规则。

## 4. 启动

### 4.0 自研 C30D 底盘驱动

当前板子是官方 RDK X5/TROS 镜像，没有 WHEELTEC 的完整 ROS2 工作空间。
`robot_base` 直接通过 C30D 的 24 字节反馈帧和 11 字节命令帧提供底盘接口：

```bash
cd ~/robot_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select robot_base robot_description
source install/setup.bash

ros2 launch robot_description description.launch.py
ros2 launch robot_base base.launch.py port:=/dev/wheeltec_controller
```

没有 udev 别名时，可以直接使用 `port:=/dev/ttyACM1`。检查：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
```

第一次测试必须架空车轮，再发送短时低速命令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

驱动在 `cmd_timeout` 后自动发送零速度。若方向相反，将 `robot_base/config/base.yaml`
中的 `invert_linear` 或 `invert_angular` 改为 `true`。

### 4.1 底盘 + 雷达 + 相机 + 视觉

```bash
ros2 launch robot_bringup robot.launch.py
```

只启动底盘：

```bash
ros2 launch robot_bringup robot.launch.py \
  use_lidar:=false use_camera:=false use_perception:=false
```

键盘遥控：

```bash
ros2 run wheeltec_robot_keyboard wheeltec_keyboard
```

### 4.2 检查话题

```bash
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /image_raw
ros2 topic echo /vision/fire_smoke/detections --once
ros2 topic echo /vision/fire_smoke/alarm
```

### 4.3 建图

```bash
ros2 launch wheeltec_slam_toolbox online_async_launch.py
ros2 launch wheeltec_nav2 save_map.launch.py
```

把保存的 `map.yaml`/`map.pgm` 放到 `~/robot_ws/maps/`。

### 4.4 导航

```bash
ros2 launch robot_navigation nav2.launch.py \
  map:=$HOME/robot_ws/maps/map.yaml
```

### 4.5 巡检任务

```bash
ros2 launch robot_mission mission.launch.py
```

默认从 `robot_mission/config/waypoints.yaml` 读取航点。先在 RViz2 里用
`2D Pose Estimate` 给 AMCL 初始位姿，再启动任务。

## 5. 视觉模型

把 `fire_smoke_yolov8n.pt` 放到：

```text
~/robot_ws/models/fire_smoke_yolov8n.pt
```

类别文件：

```text
~/robot_ws/models/fire_smoke.names
```

先跑 `.pt` 基线：

```bash
ros2 launch robot_perception fire_smoke.launch.py backend:=ultralytics
```

BPU `.bin` 的转换和适配说明见 `src/robot_perception/model/README.md`。

## 6. 开机自启动

```bash
sudo cp src/robot_bringup/systemd/robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot.service
```

如果工作区不在 `/home/root/robot_ws`，请修改 service 中的路径和 `User`。

## 7. 关键提醒

- 底盘使能开关必须 ON。
- RDK X5 使用 5V/5A 供电，不要用 PC USB 口。
- C70 先固定 640×480@30 MJPEG，不要一上来 1080p。
- N10P 是串口版，确认 `serial_port: /dev/wheeltec_lidar`。
- 差速车型的 Nav2 参数用 `param_mini_diff.yaml`，不要用 `param_mini_mec.yaml`。
- 雷达 `frame_id` 用 `laser_link`，相机 `frame_id` 用 `camera_link`，与 URDF 一致。
- URDF 当前采用 C30D 固件参数：轮径 0.065 m、轮距 0.177 m；雷达/相机高度
  按 WHEELTEC 差速平台默认值预置，装车后仍需实测复核。
