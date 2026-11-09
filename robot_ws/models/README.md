# 模型目录

把模型放在这里，不要放进源码包：

```text
models/
├── fire_smoke_yolov8n.pt
├── fire_smoke_yolov8n.onnx
├── fire_smoke_yolov8n.bin
└── fire_smoke.names
```

`fire_smoke.names` 每行一个类别名，顺序必须与训练时一致。

当前 `fire_smoke_yolov8n.pt` 已从权重元数据确认：

- `task: detect`
- `nc: 2`
- `names: {0: fire, 1: smoke}`
- `imgsz: 640`

这只确认了模型结构和类别定义；检测精度、误报率和现场泛化能力仍需用真实
图像验证。

模型转换和 BPU 适配见：

```text
src/robot_perception/model/README.md
```
