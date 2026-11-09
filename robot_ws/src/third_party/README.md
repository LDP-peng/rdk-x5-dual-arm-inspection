# Third-party packages

本目录只放从硬件资料中提取、需要随工作空间一起编译的第三方 ROS2 包。

- `lslidar_driver`：镭神 N10P 串口雷达 ROS2 驱动，来源为
  `N10系列雷达客户资料V5.0_20240822/2.ROS2_SDK/lslidar_ros2/lslidar_driver.zip`。
- `lslidar_msgs`：驱动所需的镭神自定义消息和服务，来源为同目录下的
  `lslidar_msgs.zip`。

首次编译前补齐系统依赖：

```bash
sudo apt install -y libpcl-dev libpcap-dev libboost-thread-dev libyaml-cpp-dev
```

WHEELTEC 的 `turn_on_wheeltec_robot`、`wheeltec_nav2` 等包仍然要求从
WHEELTEC ROS2 镜像或完整源码包获取；本目录不伪造这些包。
