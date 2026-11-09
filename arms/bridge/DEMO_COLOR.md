# 已弃用：官方位姿 + 蓝色识别（固定工位）

> 现场演示不能使用固定工位。动态方案见 `DEMO_VISUAL.md` 和 `visual_pick.py`。

## 原理

- 机械臂位姿完全使用 Dofbot 官方 `color_grab`：
  `look_at -> top -> blue -> 夹紧 -> top 抬起 -> gray 放置`。
- 视觉只负责确认画面中央有蓝色目标，不参与坐标换算。
- 蓝块必须放在官方固定工位，并用胶带标好位置；每次偏差控制在约 1 cm 内。
- 机械臂相机是 `/dev/video0`，图传端口是 `8082`。

官方参数：

```text
look_at = [90, 164, 18, 0, 90]
top     = [90, 80, 50, 50, 270]
blue    = [44, 66, 20, 28, 270]
gray    = [90, 48, 35, 30, 270]
HSV 蓝  = (100, 43, 46) ~ (124, 255, 255)
中央ROI = (280, 180) ~ (360, 260)
```

## 1. 检查机械臂总线

```bash
pkill -f slave_joint_node.py || true
sudo i2cdetect -y -r 5
```

必须看到 `15`。看不到时不要跑抓取，先查扩展板 12V、SDA/SCL/GND、共地。

## 2. 启动小车

终端 A：

```bash
START_FIRST_CAM=0 START_SECOND_CAM=1 DISTANCE=2.0 SPEED=0.08 bash ~/run_open_demo.sh
```

车到点停车后，不要按终端 A 的回车。

## 3. 启动从臂相机

终端 B：

```bash
nohup python3 ~/mjpeg_server.py --device /dev/video0 --port 8082 > ~/mjpeg_arm.log 2>&1 &
```

浏览器打开 `http://172.168.53.138:8082/`。

## 4. 对准蓝块

```bash
python3 ~/color_grab.py preset --name look_at
```

在 `8082` 画面里把蓝块调整到画面中央的 `80 x 80` 小区内，贴胶带固定工位。

只检测、不抓取：

```bash
python3 ~/color_grab.py detect --color blue --frames 5 --min-area 300 --roi 280,180,360,260 --source http://127.0.0.1:8082/snapshot --save ~/blue.jpg
```

最后一行必须出现 `RESULT blue ...`。

## 5. 正式抓取

手放在机械臂 12V 开关旁，执行：

```bash
python3 ~/color_grab.py auto --color blue --grip-open 1500 --grip-close 2550 --source http://127.0.0.1:8082/snapshot --device /dev/video0 --roi 280,180,360,260 --min-area 300
```

预期：回到观察位 -> 识别蓝块 -> 顶部位 -> 蓝色抓取位 -> 夹紧 -> 抬起 -> 放置 -> 回观察位。

## 6. 返航

机械臂动作结束后，回到终端 A 按一次回车，小车自动返航。

## 答辩口径

> 小车基于里程计自主驶向巡检点，激光雷达负责前向避障。到点后，后置机械臂使用官方标定位姿完成固定工位抓取，HSV 视觉负责蓝色目标确认。

不要说“端到端视觉伺服”或“相机实时定位后动态抓取”。

## 兜底

- 识别不到：确认 `8082` 能看画面、蓝块在中央 ROI、`blue.jpg` 能出结果。
- 位置识别到但夹空：蓝块偏离官方固定工位，前后左右微调胶带位置，不要改位姿。
- 夹不住：确认 `--grip-close 2550`；再大容易让舵机堵转发烫。
- I2C 报 `Remote I/O error`：先查 12V 和共地，再重跑 `i2cdetect`。
- 时间不够：先用已验证的 YOLO `model0.pt` 备份，不要现场部署 `yolov8s-worldv2.pt`。
