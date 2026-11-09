#!/usr/bin/env python3
"""Panthera-HT 主臂重力补偿节点。

主臂可自由拖动，同时把关节角发布到 ROS2，供从臂节点使用。
不依赖 Pinocchio，重力项由 urdf_gravity.py 直接解析 URDF 计算。

Ctrl+C 退出时力矩会撤销，主臂会下落，操作前必须先扶住主臂。
"""

import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from urdf_gravity import UrdfGravityModel


DEFAULT_SDK_SCRIPTS = os.path.expanduser(
    '~/arms/Panthera-HT_SDK/panthera_python/scripts'
)
DEFAULT_URDF = os.path.expanduser(
    '~/arms/Panthera-HT_SDK/panthera_python/'
    'Panthera-HT_description_V1.1.0/urdf/'
    'Panthera-HT_description_V1.1.0_leader.urdf'
)


class MasterDragNode(Node):
    def __init__(self):
        super().__init__('master_drag_node')

        self.declare_parameter('sdk_scripts_dir', DEFAULT_SDK_SCRIPTS)
        self.declare_parameter('config_path', '')
        self.declare_parameter('urdf_path', DEFAULT_URDF)
        self.declare_parameter('rate', 15.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('gravity_scale', 1.0)
        self.declare_parameter(
            'gravity_signs', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        )
        self.declare_parameter(
            'torque_limits', [15.0, 30.0, 30.0, 15.0, 5.0, 5.0]
        )
        self.declare_parameter('ramp_time', 0.8)

        self.publish_rate = float(self.get_parameter('rate').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.gravity_scale = float(self.get_parameter('gravity_scale').value)
        self.gravity_signs = np.asarray(
            self.get_parameter('gravity_signs').value, dtype=float
        )
        self.torque_limits = np.asarray(
            self.get_parameter('torque_limits').value, dtype=float
        )
        self.ramp_time = max(0.0, float(self.get_parameter('ramp_time').value))

        if self.gravity_signs.size != 6:
            raise ValueError('gravity_signs 必须包含 6 个数值')
        if self.torque_limits.size != 6:
            raise ValueError('torque_limits 必须包含 6 个数值')

        self.joint_pub = self.create_publisher(
            JointState, '/master/joint_states', 10
        )
        self.gripper_pub = self.create_publisher(
            Float32, '/master/gripper', 10
        )

        self.robot = None
        self.motors = []
        self.names = []
        self.joint_count = 0
        self.gravity_model = None
        self.control_started_at = None
        self.last_publish_at = 0.0
        self.last_log_at = 0.0
        self.last_torque = np.zeros(6, dtype=float)
        self.last_error_at = 0.0
        self.stopping = False

        self._connect_arm()

        period = 1.0 / max(self.control_rate, 1.0)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            f'主臂重力补偿已启动：控制 {self.control_rate:.0f} Hz，'
            f'发布 {self.publish_rate:.0f} Hz'
        )

    def _connect_arm(self):
        sdk_dir = os.path.expanduser(
            str(self.get_parameter('sdk_scripts_dir').value)
        )
        config_path = str(self.get_parameter('config_path').value)
        if not config_path:
            config_path = os.path.normpath(
                os.path.join(sdk_dir, '..', 'robot_param', 'Leader.yaml')
            )
        urdf_path = os.path.expanduser(
            str(self.get_parameter('urdf_path').value)
        )

        if sdk_dir not in sys.path:
            sys.path.insert(0, sdk_dir)

        import hightorque_robot as htr

        self.get_logger().info(f'加载 URDF：{urdf_path}')
        self.gravity_model = UrdfGravityModel(urdf_path)

        self.get_logger().info(f'连接主臂，配置：{config_path}')
        self.robot = htr.Robot(config_path)
        self.motors = list(self.robot.get_motors())
        self.joint_count = min(6, len(self.motors))
        if self.joint_count < 6:
            raise RuntimeError(
                f'主臂只发现 {len(self.motors)} 个电机，至少需要 6 个关节'
            )

        self.names = [f'joint{i + 1}' for i in range(len(self.motors))]
        if len(self.names) > 6:
            self.names[6] = 'gripper'

        # 先读取几帧，确保第一笔补偿力矩使用的是真实关节角。
        for _ in range(4):
            self.robot.send_get_motor_state_cmd()
            self.robot.motor_send_cmd()
            time.sleep(0.01)

        self.get_logger().info(
            f'发现 {len(self.motors)} 个电机：{self.names}'
        )
        self.get_logger().warn(
            'Ctrl+C 退出时电机会卸力，主臂会下落；退出前请先扶住主臂！'
        )

    def _read_positions(self):
        return np.asarray([
            float(motor.get_current_motor_state().position)
            for motor in self.motors
        ], dtype=float)

    def _send_commands(self, torque):
        ramp = 1.0
        if self.control_started_at is not None and self.ramp_time > 0.0:
            elapsed = time.monotonic() - self.control_started_at
            ramp = min(1.0, elapsed / self.ramp_time)

        applied = np.clip(
            torque * ramp, -self.torque_limits, self.torque_limits
        )
        self.last_torque = applied

        for index in range(self.joint_count):
            self.motors[index].pos_vel_tqe_kp_kd(
                0.0, 0.0, float(applied[index]), 0.0, 0.0
            )

        # 夹爪及额外电机保持零力矩，避免夹爪自行闭合。
        for motor in self.motors[self.joint_count:]:
            motor.pos_vel_tqe_kp_kd(0.0, 0.0, 0.0, 0.0, 0.0)

        self.robot.motor_send_cmd()

    def tick(self):
        if self.stopping:
            return

        try:
            positions = self._read_positions()
            if not np.all(np.isfinite(positions[:6])):
                raise RuntimeError('主臂关节角不是有效数值')

            gravity = self.gravity_model.gravity_torques(positions[:6])
            torque = gravity * self.gravity_signs * self.gravity_scale

            if self.control_started_at is None:
                self.control_started_at = time.monotonic()

            self._send_commands(torque)

            now = time.monotonic()
            if now - self.last_publish_at >= 1.0 / max(self.publish_rate, 1.0):
                self.last_publish_at = now
                self._publish_state(positions)

            if now - self.last_log_at >= 2.0:
                self.last_log_at = now
                values = ', '.join(f'{value:+.2f}' for value in self.last_torque)
                self.get_logger().info(f'补偿力矩 Nm：[{values}]')

        except Exception as exc:  # noqa: BLE001
            now = time.monotonic()
            if now - self.last_error_at >= 2.0:
                self.last_error_at = now
                self.get_logger().error(f'主臂控制失败：{exc}')

    def _publish_state(self, positions):
        stamp = self.get_clock().now().to_msg()

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.names
        msg.position = positions.tolist()
        self.joint_pub.publish(msg)

        if len(positions) > 6:
            gripper = Float32()
            gripper.data = float(positions[6])
            self.gripper_pub.publish(gripper)

    def stop_arm(self):
        if self.stopping:
            return
        self.stopping = True
        if self.robot is None:
            return
        try:
            self.robot.set_stop()
        except Exception:  # noqa: BLE001
            pass


def main():
    rclpy.init()
    node = None
    try:
        node = MasterDragNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop_arm()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
