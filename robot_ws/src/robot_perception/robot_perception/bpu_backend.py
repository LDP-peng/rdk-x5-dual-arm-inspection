"""BPU backend adapter.

This file is intentionally a thin adapter. The RDK X5 BPU inference pipeline
needs the pre-processing, output dequantization, decode, NMS and coordinate
scaling that are already implemented in the official example:

    /app/pydev_demo/02_detection_sample/03_ultralytics_yolov8/ultralytics_yolov8.py

Port those helpers into this class, or import them from a shared module, then
return detections in the same tuple format used by the ultralytics backend:

    (x1, y1, x2, y2, score, class_id)

The rest of the ROS 2 node is backend-agnostic, so only this file should need
to change when moving from the .pt baseline to the .bin model.
"""


class BpuBackend:
    def __init__(self, model_path, input_size, conf_threshold, iou_threshold):
        try:
            import hbm_runtime
        except ImportError as exc:
            raise RuntimeError(
                'hbm_runtime is not available. Run this node on the RDK X5 '
                'with the TROS Humble environment sourced.'
            ) from exc

        self.model_path = model_path
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.runtime = hbm_runtime.HB_HBMRuntime(model_path)

    def infer(self, frame):
        raise NotImplementedError(
            'BPU post-processing is not wired yet. Port the official '
            'ultralytics_yolov8.py pre/post-processing into '
            'robot_perception/bpu_backend.py, then implement infer().'
        )