# WHEELTEC ROS2 上板配置

WHEELTEC 的完整 ROS2 源码通常位于小车镜像中：

```text
/home/wheeltec/wheeltec_ros2
```

当前工作区里的 `6.ROS2系列教程` 是教程、PDF、视频和少量 launch 示例，不包含
完整的 `turn_on_wheeltec_robot`、`wheeltec_nav2` 源码包。不要把这些示例文件
直接当成完整包使用。

## 1. 确认源码和软件包

登录小车后执行：

```bash
ls /home/wheeltec/wheeltec_ros2/src
source /opt/tros/humble/setup.bash
ros2 pkg prefix turn_on_wheeltec_robot
ros2 pkg prefix wheeltec_nav2
```

如果 `wheeltec_ros2/src` 中存在 `turn_on_wheeltec_robot` 和
`wheeltec_nav2`，就不需要重新下载，直接使用镜像自带源码。

## 2. 设置车型和外设

编辑：

```text
/home/wheeltec/wheeltec_ros2/src/turn_on_wheeltec_robot/config/wheeltec_params.yaml
```

本项目对应配置：

```yaml
car_mode: mini_diff
ranger_avoid_flag: false
lidar_type: ls_N10P_uart
imu_model: stm32
camera_model: usb
```

说明：

- `mini_diff`：两轮差速底盘。
- `ls_N10P_uart`：N10P 串口版雷达。
- `camera_model: usb`：C70 作为普通 UVC 相机接入。
- `imu_model: stm32`：使用 C30D 上的 IMU 数据。
- 雷达串口应保持 `/dev/wheeltec_lidar`，底盘串口应为
  `/dev/wheeltec_controller`。

## 3. 编译与测试

```bash
cd /home/wheeltec/wheeltec_ros2
colcon build --symlink-install
source install/setup.bash
```

先单独测试底盘：

```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

另开终端检查：

```bash
ros2 topic hz /odom
ros2 topic hz /imu/data
ros2 topic echo /odom --once
```

再测试雷达和相机：

```bash
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
```

## 4. 与本项目的关系

WHEELTEC 负责底盘串口、`/odom`、`/imu` 和基础传感器启动；本项目负责火烟检测、
巡检状态机和 Nav2 任务封装。不要把
`turn_on_wheeltec_robot.launch.py` 与 `robot_bringup` 的总 launch 同时运行，
否则会重复创建底盘节点、EKF 或 TF。
