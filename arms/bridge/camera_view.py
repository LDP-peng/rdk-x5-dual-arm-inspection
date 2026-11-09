#!/usr/bin/env python3
"""操作台端：接收车上相机回传的 CompressedImage 并显示。

用法：
    python3 camera_view.py --ros-args -p topic:=/image

依赖：python3-opencv（sudo apt install -y python3-opencv）
按键：q 或 ESC 退出。
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraView(Node):
    def __init__(self):
        super().__init__('camera_view')
        self.declare_parameter('topic', '/image')
        self.declare_parameter('window', 'robot camera')
        self.declare_parameter('show_fps', True)

        topic = str(self.get_parameter('topic').value)
        self.window = str(self.get_parameter('window').value)
        self.show_fps = bool(self.get_parameter('show_fps').value)

        self.frames = 0
        self.fps = 0.0
        self.last_time = time.monotonic()
        self.last_warn = 0.0

        self.create_subscription(
            CompressedImage, topic, self.on_image, 10
        )
        self.get_logger().info(f'订阅 {topic}，等画面...')

    def on_image(self, msg):
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            now = time.monotonic()
            if now - self.last_warn > 2.0:
                self.last_warn = now
                self.get_logger().warn(f'解码失败，格式：{msg.format}')
            return

        self.frames += 1
        now = time.monotonic()
        if now - self.last_time >= 1.0:
            self.fps = self.frames / (now - self.last_time)
            self.frames = 0
            self.last_time = now

        if self.show_fps:
            cv2.putText(
                frame, f'{self.fps:.1f} fps', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
            )
            cv2.putText(
                frame, msg.header.frame_id, (10, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1,
            )

        cv2.imshow(self.window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            self.get_logger().info('用户退出')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
