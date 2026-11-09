#!/usr/bin/env python3
"""板A：端到端演示编排。

流程：
    1. Nav2 导航到巡检点
    2. 停车，等待操作员遥操作（回车继续）
    3. 导航回起点

用法：
    python3 mission_demo.py --ros-args \
      -p target_x:=1.0 -p target_y:=0.0 -p target_yaw:=0.0 \
      -p home_x:=0.0 -p home_y:=0.0 -p home_yaw:=0.0
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class MissionDemo(Node):
    def __init__(self):
        super().__init__('mission_demo')

        for name, default in (
            ('target_x', 1.0),
            ('target_y', 0.0),
            ('target_yaw', 0.0),
            ('home_x', 0.0),
            ('home_y', 0.0),
            ('home_yaw', 0.0),
            ('timeout', 120.0),
        ):
            self.declare_parameter(name, default)

        self.timeout = float(self.get_parameter('timeout').value)
        self.goal_timeout = 10.0
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
        pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
        return pose

    def goto(self, x, y, yaw, label):
        if not self.client.wait_for_server(timeout_sec=self.goal_timeout):
            self.get_logger().error('Nav2 未就绪，检查 nav2.launch.py 是否起来')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._pose(x, y, yaw)

        self.get_logger().info(f'{label}：({float(x):.2f}, {float(y):.2f}) yaw={float(yaw):.2f}')
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{label}：目标被拒绝')
            return False

        result_future = handle.get_result_async()
        start = time.monotonic()
        last_log = 0.0
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - start
            if elapsed - last_log >= 5.0:
                last_log = elapsed
                self.get_logger().info(f'{label} 进行中... {elapsed:.0f}s')
            if elapsed > self.timeout:
                self.get_logger().error(f'{label}：超时，取消')
                handle.cancel_goal_async()
                return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'{label}：到达')
            return True
        self.get_logger().error(f'{label}：失败，status={status}')
        return False


def main():
    rclpy.init()
    node = MissionDemo()

    target = (
        node.get_parameter('target_x').value,
        node.get_parameter('target_y').value,
        node.get_parameter('target_yaw').value,
    )
    home = (
        node.get_parameter('home_x').value,
        node.get_parameter('home_y').value,
        node.get_parameter('home_yaw').value,
    )

    try:
        if not node.goto(*target, '去巡检点'):
            return
        print('\n已到点。现在用主臂遥操作，完成后按回车返航。')
        input()
        node.goto(*home, '返航')
    except KeyboardInterrupt:
        print('\n中断。')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()