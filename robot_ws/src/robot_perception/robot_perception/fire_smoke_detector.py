import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from robot_perception.bpu_backend import BpuBackend


class UltralyticsBackend:
    def __init__(self, model_path, input_size, conf_threshold, iou_threshold, device):
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device

    def infer(self, frame):
        results = self.model.predict(
            source=frame,
            imgsz=self.input_size,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False,
        )
        if not results:
            return []

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy().astype(int)

        detections = []
        for (x1, y1, x2, y2), score, class_id in zip(xyxy, confs, classes):
            detections.append(
                (float(x1), float(y1), float(x2), float(y2), float(score), int(class_id))
            )
        return detections


class FireSmokeDetector(Node):
    def __init__(self):
        super().__init__('fire_smoke_detector')

        self.declare_parameter('backend', 'ultralytics')
        self.declare_parameter('model_path', '')
        self.declare_parameter('label_path', '')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('alert_classes', ['fire', 'smoke'])
        self.declare_parameter('alert_consecutive_frames', 3)

        self.backend_name = self.get_parameter('backend').value
        self.model_path = self.get_parameter('model_path').value
        self.label_path = self.get_parameter('label_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.input_size = int(self.get_parameter('input_size').value)
        self.device = self.get_parameter('device').value
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.alert_classes = set(self.get_parameter('alert_classes').value)
        self.alert_consecutive_frames = int(
            self.get_parameter('alert_consecutive_frames').value
        )

        self.labels = self._load_labels(self.label_path)
        self.bridge = CvBridge()
        self.backend = self._create_backend()
        self.alarm_count = 0
        self.last_log_time = 0.0

        self.det_pub = self.create_publisher(
            Detection2DArray, '/vision/fire_smoke/detections', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/vision/fire_smoke/annotated', 10
        )
        self.alarm_pub = self.create_publisher(
            Bool, '/vision/fire_smoke/alarm', 10
        )
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        self.get_logger().info(
            f'fire_smoke_detector started: backend={self.backend_name}, '
            f'model={self.model_path}, image_topic={self.image_topic}'
        )

    def _load_labels(self, path):
        if path and os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as f:
                labels = [line.strip() for line in f if line.strip()]
            if labels:
                return labels
        self.get_logger().warning(
            f'label file not found or empty: {path}; using default [fire, smoke]'
        )
        return ['fire', 'smoke']

    def _create_backend(self):
        try:
            if self.backend_name == 'ultralytics':
                return UltralyticsBackend(
                    self.model_path,
                    self.input_size,
                    self.conf_threshold,
                    self.iou_threshold,
                    self.device,
                )
            if self.backend_name == 'bpu':
                return BpuBackend(
                    self.model_path,
                    self.input_size,
                    self.conf_threshold,
                    self.iou_threshold,
                )
            raise ValueError(f'unknown backend: {self.backend_name}')
        except Exception as exc:
            self.get_logger().error(f'failed to create backend: {exc}')
            return None

    def image_callback(self, msg):
        if self.backend is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'image conversion failed: {exc}')
            return

        try:
            detections = self.backend.infer(frame)
        except Exception as exc:
            self.get_logger().error(f'inference failed: {exc}')
            return

        det_array = Detection2DArray()
        det_array.header = msg.header
        annotated = frame.copy()
        alarm = False

        for x1, y1, x2, y2, score, class_id in detections:
            label = self.labels[class_id] if 0 <= class_id < len(self.labels) else str(class_id)

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = float((x1 + x2) * 0.5)
            det.bbox.center.position.y = float((y1 + y2) * 0.5)
            det.bbox.size_x = float(max(0.0, x2 - x1))
            det.bbox.size_y = float(max(0.0, y2 - y1))

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            det_array.detections.append(det)

            if label in self.alert_classes:
                alarm = True

            if self.publish_annotated:
                color = (0, 0, 255) if label in self.alert_classes else (0, 255, 0)
                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    color,
                    2,
                )
                cv2.putText(
                    annotated,
                    f'{label} {score:.2f}',
                    (int(x1), max(0, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        self.det_pub.publish(det_array)

        if self.publish_annotated:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

        self.alarm_count = self.alarm_count + 1 if alarm else 0
        confirmed = self.alarm_count >= self.alert_consecutive_frames
        self.alarm_pub.publish(Bool(data=confirmed))

        now = self.get_clock().now().nanoseconds / 1e9
        if (det_array.detections or confirmed) and now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f'detections={len(det_array.detections)} alarm={confirmed} '
                f'count={self.alarm_count}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FireSmokeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()