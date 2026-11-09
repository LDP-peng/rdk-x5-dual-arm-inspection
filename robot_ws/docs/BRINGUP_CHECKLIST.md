# 上电检查清单

## 供电

- [ ] RDK X5 使用 5V/5A DC-DC，不用 PC USB 口
- [ ] 底盘、雷达、RDK 共地
- [ ] 电机电源与主控电源分路，DC-DC 前有保险丝
- [ ] 底盘使能开关在 ON
- [ ] 电池电压正常

## 串口

- [ ] `ls -l /dev/wheeltec_controller` 存在
- [ ] `ls -l /dev/wheeltec_lidar` 存在
- [ ] `ros2 topic hz /odom` 20~50 Hz
- [ ] `ros2 topic hz /scan` 6~12 Hz

## 相机

- [ ] `/dev/video0` 存在
- [ ] `ros2 topic hz /image_raw` 约 30 Hz
- [ ] 图像不过曝、不闪烁、不翻转

## TF

- [ ] `base_footprint → base_link → laser_link`
- [ ] `base_footprint → base_link → camera_link → camera_optical_frame`
- [ ] RViz2 中雷达点云与墙面重合

## 导航

- [ ] 地图加载成功
- [ ] AMCL 初始位姿收敛
- [ ] 单点导航成功
- [ ] 多点巡逻成功
- [ ] 障碍物前能停车/绕行

## 视觉

- [ ] `.pt` 基线能检测
- [ ] `/vision/fire_smoke/detections` 有输出
- [ ] `/vision/fire_smoke/alarm` 在连续命中后为 true
- [ ] 误报率可接受