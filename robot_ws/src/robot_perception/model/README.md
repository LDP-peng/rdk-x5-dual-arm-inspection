# 模型目录说明

把模型文件放在工作区根目录的 `models/` 下，而不是放在源码包内：

```text
~/robot_ws/models/
├── fire_smoke_yolov8n.pt       # Ultralytics / PC 基线
├── fire_smoke_yolov8n.onnx     # 量化前的 ONNX
├── fire_smoke_yolov8n.bin      # RDK X5 BPU 量化模型
└── fire_smoke.names            # 每行一个类别名
```

## 1. PC 基线

```bash
cd ~/robot_ws
python3 -c "from ultralytics import YOLO; m=YOLO('models/fire_smoke_yolov8n.pt'); print(m.names)"
```

确认类别名后，把类别写入 `models/fire_smoke.names`，例如：

```text
fire
smoke
```

## 2. 导出 ONNX

```bash
yolo export model=models/fire_smoke_yolov8n.pt format=onnx imgsz=640 opset=11 simplify=True
```

## 3. 量化成 BPU `.bin`

在 x86 开发机上用 RDK X5 OpenExplore 工具链完成。校准集建议 100~300 张真实现场图片，覆盖：

- 正常场景负样本
- 不同光照、距离、遮挡
- 火焰/烟雾的正样本

生成 `.bin` 后放到 `models/fire_smoke_yolov8n.bin`。

## 4. 板端运行

先用 Ultralytics 基线确认话题和消息链路：

```bash
ros2 launch robot_perception fire_smoke.launch.py backend:=ultralytics
```

再切换到 BPU：

```bash
ros2 launch robot_perception fire_smoke.launch.py \
  backend:=bpu \
  model_path:=$HOME/robot_ws/models/fire_smoke_yolov8n.bin
```

`backend:=bpu` 需要先把 `robot_perception/bpu_backend.py` 中的前处理、反量化、解码和 NMS
从官方示例 `/app/pydev_demo/02_detection_sample/03_ultralytics_yolov8/ultralytics_yolov8.py`
移植过来。