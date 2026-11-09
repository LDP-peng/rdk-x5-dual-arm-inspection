#!/usr/bin/env python3
"""沿机器人当前车头方向发一个 Nav2 目标。

用法：
    python3 forward_goal.py --ros-args -p distance:=1.0

为什么需要它：
    Nav2 的目标是 map 坐标系。你的走廊在地图里是斜的，
    直接发 (1.0, 0.0) 不是"车头正前方 1 米"，而是地图 +x 方向 1 米。
    这个节点先用 TF 取 map->base_link，再沿当前 yaw 前进 distance 米。
"""

import math
import time

import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ForwardGoal(Node):
    def __init__(self):
        super().__init__('forward_goal')

        self.declare_parameter('distance', 1.0)
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wait_pose', 10.0)

        self.distance = float(self.get_parameter('distance').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.frame = str(self.get_parameter('frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.wait_pose = float(self.get_parameter('wait_pose').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def current_transform(self):
        start = time.monotonic()
        while rclpy.ok():
            try:
                return self.tf_buffer.lookup_transform(
                    self.frame, self.base_frame, rclpy.time.Time())
            except Exception as exc:  # noqa: BLE001
                if time.monotonic() - start > self.wait_pose:
                    self.get_logger().error(
                        f'等不到 {self.frame}->{self.base_frame}：{exc}')
                    return None
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def run(self):
        transform = self.current_transform()
        if transform is None:
            return 1

        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        yaw = yaw_from_quaternion(transform.transform.rotation)
        goal_x = x + self.distance * math.cos(yaw)
        goal_y = y + self.distance * math.sin(yaw)

        self.get_logger().info(
            f'当前 map 位姿 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)，'
            f'目标 ({goal_x:.2f}, {goal_y:.2f})，前进 {self.distance:.2f} m')

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose 不可用，Nav2 没起来？')
            return 1

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = transform.transform.rotation

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('目标被拒绝')
            return 1

        result_future = handle.get_result_async()
        start = time.monotonic()
        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > self.timeout:
                self.get_logger().error('超时，取消')
                handle.cancel_goal_async()
                return 1

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('到达目标')
            return 0
        self.get_logger().error(f'失败，status={status}')
        return 1


def main():
    rclpy.init()
    node = ForwardGoal()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        print('\n中断。')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
