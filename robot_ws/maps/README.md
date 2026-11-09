# 地图目录

把 slam_toolbox/cartographer 保存的 `map.yaml` 和 `map.pgm` 放在这里。

导航时：

```bash
ros2 launch robot_navigation nav2.launch.py \
  map:=$HOME/robot_ws/maps/map.yaml
```

建议每次建图后记录：

- 建图区域
- 建图速度
- 是否闭环
- 已知窄道/玻璃/反光区域
- 对应 Nav2 参数文件