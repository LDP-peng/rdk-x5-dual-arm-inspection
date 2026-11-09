#!/usr/bin/env python3
"""高擎 Panthera-HT 主臂低层读数节点（不依赖 pinocchio）。

只发布 /master/joint_states，供从臂节点消费。

用法（板B）：
    python3 master_joint_node.py --ros-args \
      -p config_path:=$HOME/arms/Panthera-HT_SDK/panthera_python/robot_param/Leader.yaml

不接机械臂验证链路：
    python3 master_joint_node.py --ros-args -p mock:=true
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


DEFAULT_SDK_SCRIPTS = os.path.expanduser(
    '~/arms/Panthera-HT_SDK/panthera_python/scripts'
)


class MasterJointNode(Node):
    def __init__(self):
        super().__init__('master_joint_node')

        self.declare_parameter('sdk_scripts_dir', DEFAULT_SDK_SCRIPTS)
        self.declare_parameter('config_path', '')
        self.declare_parameter('rate', 15.0)
        self.declare_parameter('mock', False)

        self.rate = float(self.get_parameter('rate').value)
        self.mock = bool(self.get_parameter('mock').value)

        self.joint_pub = self.create_publisher(JointState, '/master/joint_states', 10)
        self.gripper_pub = self.create_publisher(Float32, '/master/gripper', 10)

        self.robot = None
        self.motors = []
        self.names = []
        self.start_time = time.monotonic()
        self.last_warn = 0.0

        if not self.mock:
            self._connect_arm()
        else:
            self.get_logger().warn('mock 模式：发布正弦轨迹，不连接机械臂')

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'主臂节点启动，{self.rate:.0f} Hz')

    def _connect_arm(self):
        sdk_dir = os.path.expanduser(
            str(self.get_parameter('sdk_scripts_dir').value)
        )
        config_path = str(self.get_parameter('config_path').value)
        if not config_path:
            config_path = os.path.normpath(
                os.path.join(sdk_dir, '..', 'robot_param', 'Leader.yaml')
            )

        if sdk_dir not in sys.path:
            sys.path.insert(0, sdk_dir)

        import hightorque_robot as htr  # 高擎低层 SDK，无需 pinocchio

        self.get_logger().info(f'连接主臂，配置：{config_path}')
        self.robot = htr.Robot(config_path)
        self.motors = list(self.robot.get_motors())
        self.names = [f'joint{i + 1}' for i in range(len(self.motors))]
        if self.names:
            self.names[-1] = 'gripper'
        self.get_logger().info(f'发现 {len(self.motors)} 个电机：{self.names}')
        self.get_logger().warn(
            'Ctrl+C 退出时电机会掉电，机械臂会下落，请先扶住！'
        )

    def _read_mock(self):
        t = time.monotonic() - self.start_time
        return [0.5 * math.sin(t + i) for i in range(6)] + [0.5 + 0.3 * math.sin(0.5 * t)]

    def _read_real(self):
        self.robot.send_get_motor_state_cmd()
        self.robot.motor_send_cmd()
        return [
            float(motor.get_current_motor_state().position)
            for motor in self.motors
        ]

    def tick(self):
        try:
            positions = self._read_mock() if self.mock else self._read_real()
        except Exception as exc:  # noqa: BLE001
            now = time.monotonic()
            if now - self.last_warn > 2.0:
                self.last_warn = now
                self.get_logger().error(f'读取主臂失败：{exc}')
            return

        if not positions:
            return

        stamp = self.get_clock().now().to_msg()

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.names
        msg.position = positions
        self.joint_pub.publish(msg)

        if len(positions) == len(self.names):
            gripper = Float32()
            gripper.data = float(positions[-1])
            self.gripper_pub.publish(gripper)


def main():
    rclpy.init()
    node = MasterJointNode()
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