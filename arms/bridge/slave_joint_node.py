#!/usr/bin/env python3
"""从臂节点：订阅 /master/joint_states，5+1 路映射，I2C 驱动亚博 Dofbot。

运行在板A（车载）。扩展板 I2C 地址 0x15，默认总线 5（RDK X5 40PIN 的 3/5 脚）。

关键安全设计：
    - 启动时用 0x31~0x36 读取六个舵机当前原始位置，作为零点；
      第一次收到主臂数据绝不会把从臂拽到固定角度。
    - 主臂超时不再发指令，从臂保持。
    - 每个舵机按自己的原始量程夹紧。
    - 先用 dry_run 验证方向和映射，再上实机。

用法：
    python3 slave_joint_node.py --ros-args -p dry_run:=true -p verbose:=true
    python3 slave_joint_node.py --ros-args -p scale:=0.3 -p verbose:=true
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import smbus
except ImportError:
    smbus = None


ADDR = 0x15
RAD2DEG = 180.0 / math.pi

# 每个舵机的原始量程（来自扩展板协议：1~4 号 900~3100，5 号 380~3700）
RAW_MIN = [900, 900, 900, 900, 380, 900]
RAW_MAX = [3100, 3100, 3100, 3100, 3700, 3100]
# 原始值 / 度
RAW_PER_DEG = [(hi - lo) / 180.0 if i != 4 else (hi - lo) / 270.0
               for i, (lo, hi) in enumerate(zip(RAW_MIN, RAW_MAX))]


def deg_to_raw(index, deg):
    """仅在读不到当前位姿时作为兜底。"""
    if index == 4:
        deg = max(0.0, min(270.0, deg))
        return int(RAW_MIN[4] + (RAW_MAX[4] - RAW_MIN[4]) * deg / 270.0)
    deg = max(0.0, min(180.0, deg))
    if index in (1, 2, 3):
        deg = 180.0 - deg
    return int(RAW_MIN[index] + (RAW_MAX[index] - RAW_MIN[index]) * deg / 180.0)


class SlaveJointNode(Node):
    def __init__(self):
        super().__init__('slave_joint_node')

        self.declare_parameter('bus', 5)
        self.declare_parameter('scale', 0.45)
        self.declare_parameter('time_ms', 100)
        self.declare_parameter('timeout', 0.35)
        self.declare_parameter('home', [90.0, 90.0, 90.0, 90.0, 90.0, 90.0])
        # Dofbot 第 5 轴是腕部自转，对应主臂第 6 轴（roll）；
        # 主臂第 5 轴是 yaw，Dofbot 没有对应自由度。
        self.declare_parameter('master_indices', [0, 1, 2, 3, 5])
        self.declare_parameter('signs', [1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('grip_index', 6)
        self.declare_parameter('grip_sign', -1.0)
        self.declare_parameter('grip_gain', 1.0)
        self.declare_parameter('grip_span', 600)
        self.declare_parameter('grip_zero_raw', -1)
        self.declare_parameter('read_home', True)
        self.declare_parameter('allow_home_fallback', False)
        self.declare_parameter('verbose', False)
        self.declare_parameter('dry_run', False)

        self.bus_num = int(self.get_parameter('bus').value)
        self.scale = float(self.get_parameter('scale').value)
        self.time_ms = int(self.get_parameter('time_ms').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.home_deg = [float(v) for v in self.get_parameter('home').value]
        self.indices = [int(v) for v in self.get_parameter('master_indices').value]
        self.signs = [float(v) for v in self.get_parameter('signs').value]
        self.grip_index = int(self.get_parameter('grip_index').value)
        self.grip_sign = float(self.get_parameter('grip_sign').value)
        self.grip_gain = float(self.get_parameter('grip_gain').value)
        self.grip_span = int(self.get_parameter('grip_span').value)
        self.grip_zero_raw = int(self.get_parameter('grip_zero_raw').value)
        self.read_home = bool(self.get_parameter('read_home').value)
        self.allow_home_fallback = bool(self.get_parameter('allow_home_fallback').value)
        self.verbose = bool(self.get_parameter('verbose').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        if len(self.indices) != 5 or len(self.signs) != 5 or len(self.home_deg) != 6:
            raise ValueError('master_indices/signs 需要 5 个，home 需要 6 个')

        self.bus = None
        self.master_home = None
        self.grip_home = None
        self.home_raw = [deg_to_raw(i, self.home_deg[i]) for i in range(6)]
        self.last_rx = 0.0
        self.stale_warned = False
        self.last_log = 0.0
        self.last_err_log = 0.0
        self.clamped_warned = False

        if self.dry_run:
            self.get_logger().warn('dry_run：只计算并打印，不写 I2C')
        else:
            self._open_bus()
            if self.read_home:
                current = self._read_positions()
                if current is not None:
                    self.home_raw = current
                    self.get_logger().info(
                        '已读取从臂当前位姿作为零点：'
                        + ', '.join(str(v) for v in current))
                elif self.allow_home_fallback:
                    self.get_logger().warn(
                        '读取从臂位姿失败，按 allow_home_fallback 使用 home 参数；'
                        '第一次接主臂前必须确认从臂就在 home 姿态')
                else:
                    self.get_logger().error(
                        '读取从臂位姿失败，拒绝启动（防止第一次就把臂拽到错误角度）。'
                        '先执行 sudo i2cdetect -y -r 5 确认 0x15；'
                        '确实要用固定 home 角再加 -p allow_home_fallback:=true')
                    raise SystemExit(1)

            if self.grip_zero_raw >= 0:
                self.home_raw[5] = max(
                    RAW_MIN[5], min(RAW_MAX[5], self.grip_zero_raw))
                self.get_logger().info(
                    f'夹爪零点固定为 raw={self.home_raw[5]}')

        self.sub = self.create_subscription(
            JointState, '/master/joint_states', self.on_master, 10
        )
        self.create_timer(0.2, self.watchdog)
        self.get_logger().info(
            f'从臂节点启动：bus={self.bus_num} addr=0x{ADDR:02X} '
            f'scale={self.scale} time={self.time_ms}ms dry_run={self.dry_run}')

    # ---------- I2C ----------
    def _open_bus(self):
        if smbus is None:
            self.get_logger().error('没有 smbus，先 sudo apt install -y python3-smbus')
            raise SystemExit(1)
        try:
            self.bus = smbus.SMBus(self.bus_num)
        except Exception as exc:
            self.get_logger().error(
                f'打不开 /dev/i2c-{self.bus_num}（{exc}），检查总线号/权限')
            raise

    def _read_positions(self):
        values = []
        for offset in range(6):
            register = 0x31 + offset
            try:
                # 与官方 Arm_Lib.Arm_serial_servo_read 保持一致：
                # RDK 的 read_word_data 返回字节序与树莓派不同，需要交换高低字节。
                self.bus.write_byte_data(ADDR, register, 0x00)
                time.sleep(0.003)
                value = self.bus.read_word_data(ADDR, register)
                value = ((value >> 8) & 0xFF) | ((value << 8) & 0xFF00)
                if value < 96 or value > 4000:
                    return None
                values.append(value)
            except Exception as exc:
                self.get_logger().warn(f'读取 {offset + 1} 号舵机失败：{exc}')
                return None
        return values

    # ---------- 主臂 ----------
    def on_master(self, msg):
        if len(msg.position) < 1:
            return

        if self.master_home is None:
            if max(self.indices) >= len(msg.position):
                self.get_logger().error(
                    f'主臂只有 {len(msg.position)} 个关节，indices={self.indices}')
                return
            self.master_home = [float(msg.position[i]) for i in self.indices]
            if self.grip_index < len(msg.position):
                self.grip_home = float(msg.position[self.grip_index])
            else:
                self.grip_home = None
            self.get_logger().info(
                '已记录主臂零点：'
                + ', '.join(f'{v:+.3f}' for v in self.master_home))
            return

        now = time.monotonic()
        self.last_rx = now
        self.stale_warned = False

        targets = []
        for out_i in range(5):
            src = self.indices[out_i]
            if src >= len(msg.position):
                targets.append(self.home_raw[out_i])
                continue
            delta = float(msg.position[src]) - self.master_home[out_i]
            if abs(delta) < 0.01:
                delta = 0.0
            raw = self.home_raw[out_i] + (
                self.signs[out_i] * self.scale * delta * RAD2DEG
                * RAW_PER_DEG[out_i])
            targets.append(raw)

        if self.grip_home is not None and self.grip_index < len(msg.position):
            delta = float(msg.position[self.grip_index]) - self.grip_home
            if abs(delta) < 0.01:
                delta = 0.0
            grip = self.home_raw[5] + (
                self.grip_sign * self.grip_gain * delta * RAD2DEG
                * RAW_PER_DEG[5])
        else:
            grip = self.home_raw[5]

        clamped = False
        for i in range(5):
            low = RAW_MIN[i] + 40
            high = RAW_MAX[i] - 40
            if targets[i] < low or targets[i] > high:
                clamped = True
            targets[i] = max(low, min(high, targets[i]))
        # Dofbot 夹爪机械行程只有约 60°~130°，不能走完整 0°~180°；
        # 以启动位姿为中心限制安全行程，避免张开过头后机构卡死。
        grip_low = max(RAW_MIN[5], self.home_raw[5] - self.grip_span)
        grip_high = min(RAW_MAX[5], self.home_raw[5] + self.grip_span)
        grip = max(grip_low, min(grip_high, grip))
        targets.append(grip)

        if clamped and not self.clamped_warned:
            self.clamped_warned = True
            self.get_logger().warn('从臂撞限位，已夹紧，建议减小 scale')
        elif not clamped:
            self.clamped_warned = False

        self._write(targets)

        if self.verbose and now - self.last_log > 1.0:
            self.last_log = now
            deltas = [targets[i] - self.home_raw[i] for i in range(6)]
            self.get_logger().info(
                '从臂 Δraw：' + ', '.join(f'{v:+6.0f}' for v in deltas))

    def _write(self, raw_values):
        if self.dry_run:
            now = time.monotonic()
            if now - self.last_log > 1.0:
                self.last_log = now
                deltas = [raw_values[i] - self.home_raw[i] for i in range(6)]
                self.get_logger().info(
                    '干跑 Δraw：' + ', '.join(f'{v:+6.0f}' for v in deltas))
            return

        data = []
        for value in raw_values:
            value = int(max(96, min(4000, value)))
            data.extend([(value >> 8) & 0xFF, value & 0xFF])
        t = self.time_ms
        try:
            self.bus.write_i2c_block_data(
                ADDR, 0x1E, [(t >> 8) & 0xFF, t & 0xFF])
            self.bus.write_i2c_block_data(ADDR, 0x1D, data)
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_err_log > 2.0:
                self.last_err_log = now
                self.get_logger().error(f'I2C 写入失败：{exc}')

    def watchdog(self):
        if self.last_rx == 0.0:
            return
        if time.monotonic() - self.last_rx > self.timeout and not self.stale_warned:
            self.stale_warned = True
            self.get_logger().warn('主臂数据超时，从臂保持当前位置')


def main():
    rclpy.init()
    node = SlaveJointNode()
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
