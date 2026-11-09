# 导航封装

本包只负责启动 Nav2，不重复启动底盘、雷达和相机。请先运行：

```bash
ros2 launch robot_bringup robot.launch.py
```

再运行导航：

```bash
ros2 launch robot_navigation nav2.launch.py \
  map:=$HOME/robot_ws/maps/map.yaml \
  params:=$HOME/robot_ws/install/wheeltec_nav2/share/wheeltec_nav2/param/wheeltec_param/param_mini_diff.yaml
```

## 建图

```bash
ros2 launch wheeltec_slam_toolbox online_async_launch.py
ros2 launch wheeltec_nav2 save_map.launch.py
```

## 差速车型参数

WHEELTEC 镜像中通常提供 `param_mini_diff.yaml`。如果实际文件名不同，用
`find ~/wheeltec_ros2 -name 'param_*diff*.yaml'` 查找，并把 `params:=` 指向它。

需要重点检查：

- `robot_radius` / `footprint` 是否与实际车体一致
- `inflation_radius` 是否留出安全距离
- `max_vel_x` / `max_vel_theta` 是否适合室内巡检
- `odom_topic`、`scan_topic`、`base_frame`、`map_frame` 是否与 TF 一致

第一版建议限速 `0.2~0.3 m/s`，确认地图和定位稳定后再提高。