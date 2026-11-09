#!/usr/bin/env python3
"""高擎 Panthera-HT 主臂 -> ROS 2 位姿桥接节点。

运行在操作台板（Ubuntu + ROS 2 Humble）上，把主臂的关节角和末端位姿
发布成 ROS 2 话题，供车上的从臂节点消费。

用法：
    python3 master_arm_node.py --ros-args \
      -p config_path:=$HOME/arms/Panthera-HT_SDK/panthera_python/robot_param/Leader.yaml

只验证网络链路、不接机械臂时：
    python3 master_arm_node.py --ros-args -p mock:=true

发布：
    /master/joint_states   sensor_msgs/JointState   6 关节 + 夹爪
    /master/eef_pose       geometry_msgs/PoseStamped 末端位姿（主臂基座系）
    /master/gripper        std_msgs/Float32          夹爪开合（rad）
"""

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


DEFAULT_SDK_SCRIPTS = os.path.expanduser(
    '~/arms/Panthera-HT_SDK/panthera_python/scripts'
)


def rotation_to_quaternion(rotation):
    """3x3 旋转矩阵（行优先）转四元数 (x, y, z, w)。"""
    m00, m01, m02 = float(rotation[0][0]), float(rotation[0][1]), float(rotation[0][2])
    m10, m11, m12 = float(rotation[1][0]), float(rotation[1][1]), float(rotation[1][2])
    m20, m21, m22 = float(rotation[2][0]), float(rotation[2][1]), float(rotation[2][2])

    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


class MasterArmNode(Node):
    def __init__(self):
        super().__init__('master_arm')

        self.declare_parameter('sdk_scripts_dir', DEFAULT_SDK_SCRIPTS)
        self.declare_parameter('config_path', '')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('frame_id', 'master_base')
        self.declare_parameter('mock', False)
        self.declare_parameter('joint_names', [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
        ])

        self.rate = float(self.get_parameter('rate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.mock = bool(self.get_parameter('mock').value)
        self.joint_names = list(self.get_parameter('joint_names').value)

        self.pose_pub = self.create_publisher(PoseStamped, '/master/eef_pose', 10)
        self.joint_pub = self.create_publisher(JointState, '/master/joint_states', 10)
        self.gripper_pub = self.create_publisher(Float32, '/master/gripper', 10)

        self.arm = None
        self.start_time = time.monotonic()
        self.last_warn = 0.0
        self.failures = 0

        if not self.mock:
            self._connect_arm()
        else:
            self.get_logger().warn('mock 模式：发布正弦轨迹，不连接机械臂')

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'主臂节点启动，{self.rate:.0f} Hz，frame={self.frame_id}'
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

        if sdk_dir not in sys.path:
            sys.path.insert(0, sdk_dir)

        try:
            from Panthera_lib import Panthera
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'无法导入 Panthera_lib（{exc}）。检查 sdk_scripts_dir 或改用 -p mock:=true'
            )
            raise

        self.get_logger().info(f'连接主臂，配置：{config_path}')
        self.arm = Panthera(config_path)
        self.get_logger().warn(
            'Ctrl+C 退出时电机会掉电，机械臂会下落，请先扶住！'
        )

    def _read_master(self):
        """返回 (关节角列表, 末端位置, 3x3 旋转矩阵, 夹爪位置)。"""
        self.arm.send_get_motor_state_cmd()
        self.arm.motor_send_cmd()

        joints = list(self.arm.get_current_pos())
        fk = self.arm.forward_kinematics()
        position = list(fk['position'])
        rotation = fk['rotation']
        gripper = float(self.arm.get_current_pos_gripper())
        return joints, position, rotation, gripper

    def _read_mock(self):
        t = time.monotonic() - self.start_time
        joints = [0.5 * math.sin(t + i) for i in range(6)]
        position = [0.30 + 0.05 * math.sin(t), 0.05 * math.cos(t), 0.25]
        rotation = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]
        gripper = 0.5 + 0.3 * math.sin(0.5 * t)
        return joints, position, rotation, gripper

    def tick(self):
        try:
            if self.mock:
                joints, position, rotation, gripper = self._read_mock()
            else:
                joints, position, rotation, gripper = self._read_master()
        except Exception as exc:  # noqa: BLE001
            self.failures += 1
            now = time.monotonic()
            if now - self.last_warn > 2.0:
                self.last_warn = now
                self.get_logger().error(
                    f'读取主臂失败（第 {self.failures} 次）：{exc}'
                )
            return

        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        qx, qy, qz, qw = rotation_to_quaternion(rotation)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        state = JointState()
        state.header.stamp = stamp
        state.name = self.joint_names + ['gripper']
        state.position = [float(v) for v in joints] + [gripper]
        self.joint_pub.publish(state)

        self.gripper_pub.publish(Float32(data=gripper))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MasterArmNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
