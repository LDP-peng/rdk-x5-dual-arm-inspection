# 双机械臂主从遥操作 — 桥接层

> **先看 `FINAL.md`，那是定稿方案。** 本文件只说明每个脚本干什么。

## 运行位置

- **板A = 车载** `192.168.166.235`：底盘、雷达、相机、从臂、导航
- **板B = 操作台** `192.168.166.207`：高擎主臂

## 当前使用

| 文件 | 板 | 作用 |
|---|---|---|
| `master_joint_node.py` | B | 读高擎 7 个电机，发 `/master/joint_states`，15 Hz |
| `slave_joint_node.py` | A | 收主臂，5+1 路映射，I2C 驱动亚博从臂 |
| `run_nav2_demo.sh` | A | 已停用：演示场地空旷，SLAM/AMCL 不可靠 |
| `run_open_demo.sh` | A | **空旷场地最终方案**：一键起车 + 前进 + 停车 + 返航 |
| `run_full_demo.sh` | A | **主从演示一键版**：先起从臂映射，再起底盘前进 + 停车 + 返航 |
| `nav2_probe.sh` | A | AMCL/Nav2 全链路诊断，输出直接发回来 |
| `odom_mission.py` | A | 保底方案：不依赖 Nav2 的巡检闭环 |
| `run_demo.sh` | A | 保底一键起底盘+雷达+图传 |
| `mjpeg_server.py` | A | MJPEG 图传，浏览器 8080 直接看 |

## 备用 / 已弃用

| 文件 | 状态 |
|---|---|
| `camera_view.py` | 板B 显示 `/image`，X11 下只有 4.8 fps，弃用 |
| `camera_pub.py` | OpenCV 发 `CompressedImage`，被 MJPEG 取代 |
| `image_compressor.py` | 同上，备用 |
| `mission_demo.py` | Nav2 版任务编排，AMCL 不通，弃用 |
| `master_arm_node.py` | 笛卡尔末端位姿版，2 天做不完，弃用 |

## 依赖

板A：

```bash
sudo apt install -y i2c-tools python3-smbus python3-opencv
```

板B 高擎 SDK 已装好，不再动。

## 快速验证

板A：

```bash
bash ~/run_demo.sh
```

板B：

```bash
python3 ~/master_joint_node.py --ros-args \
  -p config_path:=$HOME/arms/Panthera-HT_SDK/panthera_python/robot_param/Leader.yaml
```

板A 应能看到：

```bash
ros2 topic hz /master/joint_states
```

## 安全

- 主臂 Ctrl+C 前先扶住，电机会掉电。
- 从臂第一次实机 `scale:=0.15`，先 dry_run 看 `Δraw`。
- `slave_joint_node.py` 默认开机读当前位姿作零点，读不到就拒绝启动。
