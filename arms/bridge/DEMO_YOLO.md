# YOLO 抓取演示卡

## 1. 场地布置

- 车起点贴一条胶带，车头朝巡检点。
- 巡检点贴停车标记，距离用 `DISTANCE` 设定。
- 苹果核放在机械臂 `pre` 位相机的视野里，最好再贴一个物块位置标记。
- 可选：在车路径前方 0.6~0.8 m 放一个障碍物，演示雷达绕障。
- 机械臂先回到安全/直立姿态，手放在 12V 电源开关旁。

## 2. 启动车

先上传修改过的 `run_demo.sh`。它会让车只占用 `/dev/video2`，把 `/dev/video0` 留给机械臂。

终端 A：

```bash
START_FIRST_CAM=0 START_SECOND_CAM=1 DISTANCE=2.0 SPEED=0.08 bash ~/run_open_demo.sh
```

按回车开始。车会：

1. 启动底盘和雷达；
2. 沿车头前进约 2 m；
3. 前方 0.45 m 有障碍时转 60°、绕行 0.8 m；
4. 到点停车，等待终端 A 回车。

车摄像头在 `http://<板子IP>:8081/`。

## 3. 启动机械臂相机

车到点后，在终端 B：

```bash
nohup python3 ~/mjpeg_server.py --device /dev/video0 --port 8082 > ~/mjpeg_arm.log 2>&1 &
```

机械臂画面：`http://<板子IP>:8082/`。

## 4. 先彩排，不抓

```bash
python3 ~/yolo_pick.py --weights ~/model0.pt --repo ~/yolov5-6.2 --target-class Vegetable_leaf --conf 0.2 --target-cx 285 --grip-open 1500 --grip-close 2550 --dry-run
```

正常流程：`observe -> look -> pre -> J1 对准 -> 停住`。

## 5. 正式抓取

```bash
python3 ~/yolo_pick.py --weights ~/model0.pt --repo ~/yolov5-6.2 --target-class Vegetable_leaf --conf 0.2 --target-cx 285 --grip-open 1500 --grip-close 2550
```

看到机械臂夹住苹果核并抬升后，回到终端 A 按回车，车自动返航。

## 6. 演示话术

> 小车基于里程计和激光雷达前向避障，自主驶向巡检点。到点后，后置机械臂相机进行视觉检测，定位目标中心，J1 对准目标，完成夹取和抬升。操作员确认后，小车自动返航。

注意：这个演示里 YOLO 的类别名只是触发信号，准确说法是“视觉检测定位后抓取”，不要说“模型准确识别苹果核”。

## 7. 兜底

- J1 越转越偏：命令最后加 `--align-sign -1`。
- 识别不到：先降低 `--conf 0.1`，或把苹果核往画面中心/近处挪。
- 识别不稳定：直接加 `--fixed`，用胶带固定物块位置，走固定工位抓取。
- 车停得不够准：减小 `DISTANCE` 或重新贴停车标记，先空跑确认停车位置。
- 车图传不需要：把 `START_SECOND_CAM=1` 改成 `0`，减少设备占用。
