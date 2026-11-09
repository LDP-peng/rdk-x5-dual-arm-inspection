# 终极方案 V3（空旷场地：odom）

## 一句话

**演示场地空旷，SLAM/AMCL 不可靠，已确定走 B 案：`odom` 航点 + `/scan` 急停。演示流程不变，不再建图。**

一键启动：

```bash
bash ~/run_open_demo.sh
```

默认前进 1.5 m，到点停车等待主臂遥操作，按回车返航。现场空间不够就缩小距离：

```bash
DISTANCE=1.0 SPEED=0.08 bash ~/run_open_demo.sh
```

---

## 1. A 案：Nav2（已停用：场地空旷）

### 启动

先把车放回**建图起点**，车头朝建图时的方向，然后：

```bash
bash ~/run_nav2_demo.sh
```

默认初始位姿 `(0, 0, 0)`。如果起点不在建图原点：

```bash
INIT_X=0.5 INIT_Y=-0.2 INIT_YAW=0 bash ~/run_nav2_demo.sh
```

脚本会自动：

1. 后台起底盘 + 雷达，检查 `/odom`、`/scan`
2. 后台起 `map_server + AMCL + planner + controller`
3. 等四个节点全部 `active`
4. 发 `/initialpose`
5. 12 秒内看 `/amcl_pose` 和 `map->odom`

### AMCL 四道门槛

| # | 检查 | 通过标准 |
|---|---|---|
| 1 | `/scan` | 10 Hz，`header.stamp` 是当前时间 |
| 2 | TF | `odom→base_footprint→base_link→laser_link` 全通 |
| 3 | `/map` | AMCL 能收到，分辨率 0.05 |
| 4 | 初始位姿 | 发出后 12 秒内出 `/amcl_pose`，`map→odom` 有数 |

四道全过，才算“保留成功”。

### 没过怎么办

脚本默认 `FALLBACK_STATIC=1`：自动杀掉 AMCL，起静态 `map→odom`。
**Nav2 的全局规划、代价地图、控制器全部保留**，只是定位不再由 AMCL 提供。

不想要自动切换、只想看诊断：

```bash
FALLBACK_STATIC=0 bash ~/run_nav2_demo.sh
```

或者单独跑：

```bash
bash ~/nav2_probe.sh
```

把输出整段发回来。

### 导航测试

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 2. B 案：odom 航点（最终方案）

A 案 90 分钟没过，直接切这个，不再碰 AMCL。

```bash
bash ~/run_demo.sh
python3 ~/odom_mission.py --ros-args -p target_x:=1.0 -p target_y:=0.0
```

这个方案 10 分钟内能出结果，演示流程与 A 案完全一样。

---

## 3. 时间表（48 小时）

| 时间 | 任务 | 过关标准 |
|---|---|---|
| 0~1.5 h | A 案 Nav2 + AMCL 四检 | `/amcl_pose` 有数，`map→odom` 有数 |
| 没过 | 立刻切 B 案 | 车能去 1 m 再回来 |
| 1.5~3 h | 图传 + 主臂跨板 | 8080 能看，`/master/joint_states` 15 Hz |
| 3~4 h | 端到端彩排 3 次 | 去点→停车→回车→返航 |
| 睡 4 h | — | — |
| 线到后 2~3 h | 从臂接线 + dry_run + 实机 | 主臂动，从臂跟 |
| 最后 2 h | 全流程彩排、录像 | 连续 3 次不翻车 |

---

## 4. 部署

Windows PowerShell：

```powershell
scp "E:\RDK X5\arms\bridge\run_nav2_demo.sh" "E:\RDK X5\arms\bridge\nav2_probe.sh" "E:\RDK X5\arms\bridge\run_demo.sh" "E:\RDK X5\arms\bridge\odom_mission.py" "E:\RDK X5\arms\bridge\slave_joint_node.py" sunrise@192.168.166.235:~/
```

两台板子统一环境：

```bash
grep -q 'ROS_DOMAIN_ID=0' ~/.bashrc || echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
source ~/.bashrc
```

---

## 5. 从臂（杜邦线到货后）

| 扩展板 | RDK X5 40PIN |
|---|---|
| SDA | pin 3 |
| SCL | pin 5 |
| GND | pin 6 |

**5V 绝对不接。**

```bash
sudo apt install -y i2c-tools python3-smbus
sudo i2cdetect -y -r 5
```

看到 `15` 再继续：

```bash
python3 ~/slave_joint_node.py --ros-args -p dry_run:=true -p verbose:=true -p scale:=0.3
python3 ~/slave_joint_node.py --ros-args -p scale:=0.15 -p verbose:=true
```

调参顺序：先确认 `master_indices` 顺序，再调 `signs`，最后把 `scale` 从 `0.15` 加到 `0.45`。

---

## 6. 环境

**不要再装 ROS、Nav2、Pinocchio。** 现在只缺从臂的包：

```bash
sudo apt install -y i2c-tools python3-smbus
```

地图：`~/robot_ws/maps/nav_map.yaml`，分辨率 0.05。

---

## 7. 演示话术

1. “小车用激光 SLAM 建好的地图，自主规划路径驶入模拟机柜点位。”
2. “车载相机把舱内画面实时回传到远端操作台。”
3. “操作员拖动桌面主臂，车载从臂同步复刻姿态，完成接触式点检，然后自动返航。”

如果评委追问定位：

- AMCL 正常：直接说“AMCL 在地图内实时定位，Nav2 全局规划”。
- 走了静态保底：说“现场起点固定，`map→odom` 用固定变换，Nav2 仍按地图做全局规划；AMCL 重定位是下一步”。

---

## 8. 兜底

- Nav2 起不来：`tail -60 /tmp/nav2.log`。
- AMCL 没位姿：`FALLBACK_STATIC=0 bash ~/run_nav2_demo.sh` 后跑 `nav2_probe.sh`。
- 车不走：`bash ~/run_demo.sh status`，先看 `/odom`、`/scan`。
- 从臂不动：`sudo i2cdetect -y -r 5` 没有 `15` 就是接线问题，不要改代码。
