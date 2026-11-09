#!/usr/bin/env python3
"""板A：把 /image 压成 JPEG 后发 /image/compressed，供板B 显示。

用法：
    python3 image_compressor.py --ros-args -p quality:=70 -p max_fps:=15.0
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')

        self.declare_parameter('in_topic', '/image')
        self.declare_parameter('out_topic', '/image/compressed')
        self.declare_parameter('quality', 70)
        self.declare_parameter('max_fps', 15.0)

        in_topic = str(self.get_parameter('in_topic').value)
        out_topic = str(self.get_parameter('out_topic').value)
        self.quality = int(self.get_parameter('quality').value)
        self.min_period = 1.0 / max(1.0, float(self.get_parameter('max_fps').value))

        self.bridge = CvBridge()
        self.last = 0.0
        self.frames = 0
        self.bytes = 0
        self.last_report = time.monotonic()

        self.pub = self.create_publisher(CompressedImage, out_topic, 1)
        self.sub = self.create_subscription(Image, in_topic, self.on_image, 1)
        self.get_logger().info(
            f'{in_topic} -> {out_topic}, quality={self.quality}, '
            f'max_fps={1.0 / self.min_period:.0f}'
        )

    def on_image(self, msg):
        now = time.monotonic()
        if now - self.last < self.min_period:
            return
        self.last = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge 转换失败：{exc}')
            return

        ok, buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        )
        if not ok:
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub.publish(out)

        self.frames += 1
        self.bytes += len(out.data)
        if now - self.last_report >= 2.0:
            fps = self.frames / (now - self.last_report)
            kbps = self.bytes / (now - self.last_report) / 1024.0
            self.get_logger().info(f'{fps:.1f} fps, {kbps:.0f} KB/s')
            self.frames = 0
            self.bytes = 0
            self.last_report = now


def main():
    rclpy.init()
    node = ImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()