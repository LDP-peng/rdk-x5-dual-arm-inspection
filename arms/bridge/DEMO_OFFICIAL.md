# 官方位姿动态抓取：top 观察 + blue 下降

## 前提

- 机械臂 I2C 地址 `0x15` 正常；
- `/dev/video0` 的 8082 图传正常；
- 不需要 `grab_poses.json`，不需要示教 pre/grasp/lift。

```bash
pkill -f slave_joint_node.py || true
sudo i2cdetect -y -r 5
```

## 一次性标定夹爪像素

```bash
python3 ~/official_pick.py calibrate --color blue --scan-pose top --grasp-pose blue
```

脚本会：

1. 先移动到官方 `blue` 抓取位；
2. 提示你把蓝块放到夹爪能夹住的位置；
3. 升到官方 `top`；
4. 记录蓝块在 top 画面里的 `target_cx/target_cy`。

如果 top 画面看不到蓝块，改用官方 `look_at` 作为观察位：

```bash
python3 ~/official_pick.py calibrate --color blue --scan-pose look_at --grasp-pose blue
```

## 换位置干跑

把蓝块横移 5–10 cm：

```bash
python3 ~/official_pick.py grab --color blue --radial-joint 3 --dry-run
```

它会用 J1/J3 把蓝块拉回标定像素，并打印 `视觉偏移`。

## 正式抓取

```bash
python3 ~/official_pick.py grab --color blue --radial-joint 3 --grip-open 1500 --grip-close 2550
```

执行顺序：

```text
官方 top 观察 -> J1/J3 视觉对准 -> 官方 blue + 偏移下降 -> 夹紧 -> 回 top 抬起
```

## 调试

- top 看不到目标：在 calibrate 和 grab 都加 `--scan-pose look_at`。
- J3 修正弱：改 `--radial-joint 2`。
- 目标丢失：减小 `--probe-step 20`，并确保蓝块完整在画面内。
- 对准后夹偏：用 `--target-cx`、`--target-cy` 微调 5–10 px。

边界：物块必须落在观察位相机视野内；官方 blue 位是基础抓取姿态，最终位置由视觉偏移修正。
