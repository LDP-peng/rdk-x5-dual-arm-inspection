#!/usr/bin/env python3
"""板A：OpenCV 直接抓 USB 相机，发 CompressedImage（绕开 hobot_usb_cam）。

用法：
    python3 camera_pub.py --ros-args -p device:=/dev/video0 -p width:=640 -p height:=480
"""

import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraPub(Node):
    def __init__(self):
        super().__init__('camera_pub')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('quality', 70)
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('topic', '/image/compressed')
        self.declare_parameter('frame_id', 'camera_link')

        device = str(self.get_parameter('device').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        self.quality = int(self.get_parameter('quality').value)
        self.min_period = 1.0 / max(1.0, float(self.get_parameter('max_fps').value))
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.pub = self.create_publisher(
            CompressedImage, str(self.get_parameter('topic').value), 1
        )

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f'打不开相机 {device}')
            raise SystemExit(1)

        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().error(f'{device} 打不开画面，检查设备号/占用')
            raise SystemExit(1)
        self.get_logger().info(
            f'{device} 已打开，实际 {frame.shape[1]}x{frame.shape[0]}'
        )

        self.last = 0.0
        self.frames = 0
        self.bytes = 0
        self.last_report = time.monotonic()
        self.timer = self.create_timer(1.0 / max(1.0, float(self.get_parameter('max_fps').value)), self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        stamp = self.get_clock().now().to_msg()

        ok, buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        )
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.pub.publish(msg)

        now = time.monotonic()
        self.frames += 1
        self.bytes += len(msg.data)
        if now - self.last_report >= 2.0:
            dt = now - self.last_report
            self.get_logger().info(
                f'{self.frames / dt:.1f} fps, {self.bytes / dt / 1024.0:.0f} KB/s'
            )
            self.frames = 0
            self.bytes = 0
            self.last_report = now


def main():
    rclpy.init()
    node = CameraPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()