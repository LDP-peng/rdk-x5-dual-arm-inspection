#!/usr/bin/env python3
"""HSV 视觉对准：居中积木并前进到固定画面面积后停车。

用法：
    python3 ~/visual_align.py --ros-args -p color:=blue -p target_area:=7000

前提：8081 图传在跑，积木在车头前方，/scan 正常。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import color_grab as cg


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class VisualAlign(Node):
    def __init__(self):
        super().__init__('visual_align')
        for name, value in (
            ('color', 'blue'),
            ('target_cx', 320.0),
            ('target_area', 7000.0),
            ('min_area', 300.0),
            ('center_tol', 40.0),
            ('max_linear', 0.05),
            ('max_angular', 0.30),
            ('scan_stop', 0.28),
            ('timeout', 15.0),
            ('invert_turn', False),
            ('dry_run', False),
        ):
            self.declare_parameter(name, value)

        self.color = str(self.get_parameter('color').value)
        self.target_cx = float(self.get_parameter('target_cx').value)
        self.target_area = float(self.get_parameter('target_area').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.center_tol = float(self.get_parameter('center_tol').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.scan_stop = float(self.get_parameter('scan_stop').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.invert_turn = bool(self.get_parameter('invert_turn').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.scan = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.source = cg.FrameSource(
            'http://127.0.0.1:8081/snapshot', '/dev/video2')

    def on_scan(self, msg):
        self.scan = msg

    def front_range(self):
        if self.scan is None:
            return None
        best = 999.0
        scan = self.scan
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            angle = wrap(scan.angle_min + index * scan.angle_increment)
            if abs(angle) <= math.radians(25.0):
                best = min(best, value)
        return best

    def stop(self):
        if not self.dry_run:
            self.cmd_pub.publish(Twist())

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        if not self.dry_run:
            self.cmd_pub.publish(msg)
        print(f'cmd linear={linear:.3f} angular={angular:.3f}', flush=True)

    def run(self):
        start = time.monotonic()
        front = self.front_range()
        if front is None:
            self.get_logger().error('/scan 没有数据，退出')
            self.stop()
            return 2
        if front < self.scan_stop:
            self.get_logger().error(f'前方太近：{front:.2f} m，退出')
            self.stop()
            return 2

        turn_sign = 1.0 if self.invert_turn else -1.0
        missing = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.monotonic() - start > self.timeout:
                print('超时，停车', flush=True)
                self.stop()
                return 1

            frame = self.source.read()
            found = None if frame is None else cg.detect(
                frame, self.color, self.min_area)
            if found is None:
                missing += 1
                self.stop()
                if missing >= 12:
                    print('连续没有目标，停车', flush=True)
                    return 2
                time.sleep(0.15)
                continue

            missing = 0
            name, area, cx, cy, _ = found
            if self.color != 'any' and name != self.color:
                print(f'颜色不对：看到 {name}，停车', flush=True)
                self.stop()
                return 2

            error_x = cx - self.target_cx
            centered = abs(error_x) <= self.center_tol
            close_enough = area >= self.target_area
            print(
                f'{name} area={int(area)} cx={cx} cy={cy} '
                f'err_x={error_x:.0f}', flush=True)

            if centered and close_enough:
                self.stop()
                print('ALIGNED：已居中且到达目标距离', flush=True)
                return 0

            if not centered:
                angular = turn_sign * 0.004 * error_x
                angular = max(-self.max_angular, min(self.max_angular, angular))
                self.drive(0.0, angular)
            else:
                front = self.front_range()
                if front is None or front < self.scan_stop:
                    self.stop()
                    print('前方过近，停车', flush=True)
                    return 2
                linear = 0.02 + 0.03 * max(
                    0.0, 1.0 - area / self.target_area)
                linear = max(0.015, min(self.max_linear, linear))
                angular = turn_sign * 0.002 * error_x
                angular = max(-0.12, min(0.12, angular))
                self.drive(linear, angular)

            time.sleep(0.12)

        self.stop()
        return 1


def main():
    rclpy.init()
    node = VisualAlign()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
