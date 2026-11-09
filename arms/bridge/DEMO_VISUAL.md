# 动态视觉抓取：标定 + 二维视觉伺服

## 重新示教 pre / grasp / lift

```bash
pkill -f slave_joint_node.py || true
python3 ~/arm_teach.py
```

在 `cmd>` 里依次操作：

1. 张开夹爪：`open`。
2. 把机械臂移到物块上方，镜头俯视且 8082 能看到完整蓝块：`save pre`。
3. 只降低 J2/J3，让夹爪进到物块两侧，其他关节尽量不动：`save grasp`。
4. 夹紧、抬起：`close`，再上升：`save lift`。
5. 退出前用 `pose pre`、`pose grasp`、`pose lift` 各回放一次，确认动作是纯下降/纯抬起。

关键：`pre` 必须能看到蓝块；`pre -> grasp` 只允许下降，不要让 J1 或腕部姿态变化太大。

## 解决什么问题

相机俯下时比夹爪前探约 5.5 cm。直接看颜色再下降到固定点会夹空。

本方案不写死蓝块坐标：

1. 一次性记录“夹爪正对蓝块时”的画面像素；
2. 演示时蓝块在 pre 相机视野内换位置；
3. J1/J3 自动把蓝块对到该像素；
4. 相机位姿与标定时一致，再执行相对下降夹取。

边界：蓝块必须落在 pre 位相机可达视野内，不是全桌面任意位置。

## 1. 准备

```bash
pkill -f slave_joint_node.py || true
sudo i2cdetect -y -r 5
nohup python3 ~/mjpeg_server.py --device /dev/video0 --port 8082 > ~/mjpeg_arm.log 2>&1 &
```

`i2cdetect` 必须看到 `15`。

## 2. 一次性标定

把蓝块放到**夹爪真正能夹住的位置**。然后在终端执行：

```bash
python3 ~/visual_pick.py calibrate --color blue --min-area 120 --grip-open 1500 --save ~/visual_calib.jpg
```

它会先移动到 `pre`，记录蓝块在这个位置的像素 `target_cx/target_cy`。

标定文件：`~/visual_pick_calib.json`

如果提示目标太靠近边缘，把扫视/观察位稍微抬高后重新标定。

## 3. 换位置做干跑

把蓝块向左或向右移动 5–10 cm，再执行：

```bash
python3 ~/visual_pick.py grab --color blue --radial-joint 3 --dry-run
```

正常表现：

- 自动搜索到蓝块；
- 打印局部雅可比；
- J1/J3 逐步把 `cx/cy` 拉向标定像素；
- 显示“已对准夹爪标定像素”。

## 4. 正式抓取

```bash
python3 ~/visual_pick.py grab --color blue --radial-joint 3 --grip-open 1500 --grip-close 2550
```

## 5. 调试

- J3 探测弱或目标丢失：换 `--radial-joint 2`。
- 对准慢或来回摆：减小 `--max-j1-step 35 --max-radial-step 30`。
- 搜索范围不够：逐步增大 `--scan-count`，不要一次调太大。
- 边缘检测不稳：减小 `--probe-step 20`，并保证蓝块完整在画面内。
- 对准后仍夹偏：用 `--target-cx` / `--target-cy` 微调 5–10 px，再重标一次。

## 6. 演示口径

> 机械臂先俯视搜索蓝色目标，基于像素闭环修正 J1 和俯仰关节，把目标对准夹爪标定位置后再下降抓取。视觉参与了实时搜索和位置修正，目标是动态放置的。

不要说“端到端视觉伺服”或“全桌面任意抓取”。
