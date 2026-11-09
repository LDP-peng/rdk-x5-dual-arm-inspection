#!/usr/bin/env python3
"""板A：不依赖 Nav2 / AMCL 的巡检闭环。

流程：
    起点 -> 相对航点（巡检点）-> 停车等待遥操作 -> 回车 -> 返回起点

定位用的是底盘 /odom（20 Hz，短距离误差可接受），避障用 /scan 前向急停。
地图只用于展示，不参与控制，所以不会出现 AMCL 不发布位姿导致整套卡死。

用法：
    python3 odom_mission.py --ros-args \
      -p target_x:=1.0 -p target_y:=0.0 -p target_yaw:=0.0

    # 到巡检点后自动回放 ~/grab_poses.json 中的示教位姿
    python3 odom_mission.py --ros-args -p arm_demo:=true

    # 只跑过去、不等待、不返航（调试用）
    python3 odom_mission.py --ros-args -p pause:=false -p return_home:=false

    # 干跑，只打印不发给底盘
    python3 odom_mission.py --ros-args -p dry_run:=true

新增参数：
    dynamic_near / dynamic_closing   目标靠近到什么距离、多快时停车等待
    resume_dist / resume_hold        让开到多远、持续多久后自动恢复
    wait_timeout                     等待超时后报警，按 r 人工放行
    side_block                       左、右都小于它时不绕行，停车报警
    alarm_script                     报警时执行的外部命令（例如蜂鸣器脚本）
"""

import math
import os
import shlex
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class KeyPoller:
    """Linux 终端下单键非阻塞读取；不是终端时自动禁用。

    cbreak 模式下按键立即生效、不用回车；退出时恢复终端设置，
    所以后面 pause_for_operator 里的 input() 不受影响。
    """

    def __init__(self):
        self._termios = None
        self._saved = None
        self._fd = None
        self.enabled = False

    def __enter__(self):
        try:
            import termios
            import tty
            if not sys.stdin.isatty():
                return self
            self._termios = termios
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self.enabled = True
        except Exception:
            self.enabled = False
        return self

    def __exit__(self, *_):
        if self.enabled and self._saved is not None:
            try:
                self._termios.tcsetattr(
                    self._fd, self._termios.TCSADRAIN, self._saved)
            except Exception:
                pass
        self.enabled = False
        return False

    def read(self, timeout=0.0):
        if not self.enabled:
            return None
        try:
            import select
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if not ready:
                return None
            return sys.stdin.read(1)
        except Exception:
            return None


class OdomMission(Node):
    def __init__(self):
        super().__init__('odom_mission')

        params = (
            ('target_x', 1.0),
            ('target_y', 0.0),
            ('target_yaw', 0.0),
            ('home_x', 0.0),
            ('home_y', 0.0),
            ('home_yaw', 0.0),
            ('pause', True),
            ('return_home', True),
            ('dry_run', False),
            ('max_linear', 0.12),
            ('max_angular', 0.55),
            ('position_tolerance', 0.05),
            ('yaw_tolerance', 0.07),
            ('front_stop', 0.38),
            ('front_half_angle', 0.55),
            ('front_slow', 0.75),
            ('avoidance', True),
            ('avoid_clear', 0.75),
            ('avoid_turn_deg', 50.0),
            ('avoid_turn_speed', 0.35),
            ('avoid_turn_timeout', 9.0),
            ('avoid_forward_speed', 0.08),
            ('avoid_forward_distance', 1.0),
            ('avoid_forward_timeout', 12.0),
            ('avoid_max_tries', 3),
            ('obstacle_wait', 6.0),
            ('dynamic_near', 0.60),
            ('dynamic_closing', 0.15),
            ('resume_dist', 0.80),
            ('resume_hold', 2.0),
            ('wait_timeout', 25.0),
            ('side_block', 0.30),
            ('alarm_script', ''),
            ('leg_timeout', 70.0),
            ('arm_demo', False),
            ('arm_demo_script', '~/arm_pose_demo.py'),
            ('arm_demo_poses', 'auto'),
            ('arm_demo_time_ms', 1400),
        )
        for name, default in params:
            self.declare_parameter(name, default)

        self.target = (
            float(self.get_parameter('target_x').value),
            float(self.get_parameter('target_y').value),
            float(self.get_parameter('target_yaw').value),
        )
        self.home = (
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_yaw').value),
        )
        self.do_pause = bool(self.get_parameter('pause').value)
        self.return_home = bool(self.get_parameter('return_home').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.pos_tol = float(self.get_parameter('position_tolerance').value)
        self.yaw_tol = float(self.get_parameter('yaw_tolerance').value)
        self.front_stop = float(self.get_parameter('front_stop').value)
        self.front_half_angle = float(self.get_parameter('front_half_angle').value)
        self.front_slow = float(self.get_parameter('front_slow').value)
        self.avoidance = bool(self.get_parameter('avoidance').value)
        self.avoid_clear = float(self.get_parameter('avoid_clear').value)
        self.avoid_turn_deg = float(
            self.get_parameter('avoid_turn_deg').value)
        self.avoid_turn_speed = float(
            self.get_parameter('avoid_turn_speed').value)
        self.avoid_turn_timeout = float(
            self.get_parameter('avoid_turn_timeout').value)
        self.avoid_forward_speed = float(
            self.get_parameter('avoid_forward_speed').value)
        self.avoid_forward_distance = float(
            self.get_parameter('avoid_forward_distance').value)
        self.avoid_forward_timeout = float(
            self.get_parameter('avoid_forward_timeout').value)
        self.avoid_max_tries = int(self.get_parameter('avoid_max_tries').value)
        self.obstacle_wait = float(self.get_parameter('obstacle_wait').value)
        self.dynamic_near = float(self.get_parameter('dynamic_near').value)
        self.dynamic_closing = float(
            self.get_parameter('dynamic_closing').value)
        self.resume_dist = float(self.get_parameter('resume_dist').value)
        self.resume_hold = float(self.get_parameter('resume_hold').value)
        self.wait_timeout = float(self.get_parameter('wait_timeout').value)
        self.side_block = float(self.get_parameter('side_block').value)
        self.alarm_script = str(self.get_parameter('alarm_script').value)
        self.leg_timeout = float(self.get_parameter('leg_timeout').value)
        self.arm_demo = bool(self.get_parameter('arm_demo').value)
        self.arm_demo_script = os.path.expanduser(
            str(self.get_parameter('arm_demo_script').value))
        self.arm_demo_poses = str(
            self.get_parameter('arm_demo_poses').value)
        self.arm_demo_time_ms = int(
            self.get_parameter('arm_demo_time_ms').value)

        self.pose = None
        self.scan = None
        self.last_scan_time = 0.0
        self.last_log_time = 0.0
        self.last_linear = 0.0
        self.side_override = 0

        self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.dry_run:
            self.get_logger().warn('dry_run：只打印，不发 /cmd_vel')

    # ---------- 回调 ----------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q))

    def on_scan(self, msg):
        self.scan = msg
        self.last_scan_time = time.monotonic()

    # ---------- 基础 ----------
    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)

    def publish(self, linear, angular):
        if self.dry_run:
            return
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.last_linear = 0.0
        for _ in range(3):
            self.publish(0.0, 0.0)
            self.tick()
        if not self.dry_run:
            self.publish(0.0, 0.0)

    def wait_for_odom(self, timeout=15.0):
        start = time.monotonic()
        while self.pose is None and rclpy.ok():
            if time.monotonic() - start > timeout:
                self.get_logger().error('/odom 15 秒没有数据，先检查底盘驱动')
                return False
            self.tick()
        return True

    def wait_for_scan(self, timeout=10.0):
        start = time.monotonic()
        while self.scan is None and rclpy.ok():
            if time.monotonic() - start > timeout:
                self.get_logger().error('/scan 10 秒没有数据，先检查雷达驱动')
                return False
            self.tick()
        return True

    def front_range(self):
        if self.scan is None:
            return math.inf
        best = math.inf
        scan = self.scan
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            angle = wrap(scan.angle_min + index * scan.angle_increment)
            if abs(angle) <= self.front_half_angle:
                best = min(best, value)
        return best

    def sector_range(self, low_deg, high_deg):
        """返回指定角度扇区内的最近距离（米）。"""
        if self.scan is None:
            return math.inf
        low = math.radians(low_deg)
        span = math.radians((high_deg - low_deg) % 360.0)
        best = math.inf
        scan = self.scan
        for index, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            angle = wrap(scan.angle_min + index * scan.angle_increment)
            relative = (angle - low) % (2.0 * math.pi)
            if relative <= span:
                best = min(best, value)
        return best

    # ---------- 报警与等待 ----------
    def alarm(self, text):
        """报警：终端响铃 + 醒目文字框 + 可选外部命令（接真蜂鸣器）。"""
        self.get_logger().error(text)
        print('\a' * 3, end='', flush=True)
        print('=' * 64)
        print('  ' + text)
        print('=' * 64, flush=True)
        if self.alarm_script:
            try:
                subprocess.Popen(shlex.split(self.alarm_script))
            except Exception as exc:
                self.get_logger().error(f'报警脚本启动失败：{exc}')

    def hold_still(self, label, secs=2.0):
        """原地静止一段时间，期间持续发 0 速度。"""
        start = time.monotonic()
        while time.monotonic() - start < secs and rclpy.ok():
            self.publish(0.0, 0.0)
            self.tick()
        self.get_logger().info(f'{label}：原地观察 {secs:.0f} 秒结束')

    def wait_for_traffic(self, label, front):
        """动态目标：停车让行，让开 0.8 m 持续 2 秒自动走，超时报警等 r。"""
        self.stop()
        self.get_logger().warn(
            f'{label}：前方 {front:.2f} m 有目标靠近，停车等待让行')
        start = time.monotonic()
        clear_since = None
        next_alarm = start + self.wait_timeout

        with KeyPoller() as keys:
            if not keys.enabled:
                self.get_logger().warn(
                    '当前不是交互终端，按 r 不可用，只能等目标自动让开')
            while rclpy.ok():
                self.publish(0.0, 0.0)
                self.tick()
                now = time.monotonic()

                if now - self.last_scan_time > 1.5:
                    self.stop()
                    self.get_logger().error(
                        f'{label}：等待时 /scan 超过 1.5 秒没数据，停车')
                    return False

                if keys.read(0.0) in ('r', 'R'):
                    self.get_logger().warn(f'{label}：人工确认，结束等待')
                    self.hold_still(label)
                    return True

                front = self.front_range()
                if front > self.resume_dist:
                    if clear_since is None:
                        clear_since = now
                    elif now - clear_since >= self.resume_hold:
                        self.get_logger().info(
                            f'{label}：前方已让开到 {front:.2f} m，恢复前进')
                        return True
                else:
                    clear_since = None

                if now >= next_alarm:
                    next_alarm = now + 15.0
                    self.alarm(
                        f'{label}：通道受阻超过 {self.wait_timeout:.0f} 秒，'
                        '请清理通道后按 r 继续')
        return False

    def wait_side_blocked(self, label, front, left, right):
        """左右都堵死：不绕行，停车报警，等人工按 r 强制放行一次。"""
        self.stop()
        self.alarm(
            f'{label}：前方 {front:.2f} m 有障碍，左右只有 '
            f'{left:.2f}/{right:.2f} m，不绕行，停车报警')

        with KeyPoller() as keys:
            if not keys.enabled:
                self.get_logger().warn(
                    '当前不是交互终端，按 r 不可用，用 Ctrl+C 退出')
            next_alarm = time.monotonic() + 15.0
            while rclpy.ok():
                self.publish(0.0, 0.0)
                self.tick()
                now = time.monotonic()

                if now - self.last_scan_time > 1.5:
                    self.stop()
                    self.get_logger().error(
                        f'{label}：等待时 /scan 超过 1.5 秒没数据，停车')
                    return False

                if keys.read(0.0) in ('r', 'R'):
                    self.get_logger().warn(
                        f'{label}：人工确认，允许强制绕行一次')
                    self.hold_still(label)
                    return 'manual'

                left = self.sector_range(45.0, 135.0)
                right = self.sector_range(-135.0, -45.0)
                if left >= self.side_block or right >= self.side_block:
                    self.get_logger().info(
                        f'{label}：侧向已空出（左 {left:.2f} m、'
                        f'右 {right:.2f} m），恢复绕行')
                    return 'clear'

                if now >= next_alarm:
                    next_alarm = now + 15.0
                    self.alarm(
                        f'{label}：通道仍被堵死，请清理后按 r 强制绕行')
        return False

    def avoid_obstacle(self, label, front, goal_x, goal_y):
        """绕障：向空间更大的一侧转 60 度，侧向前进后再回目标。"""
        # LaserScan 角度向右为负、向左为正。
        left = self.sector_range(45.0, 135.0)
        right = self.sector_range(-135.0, -45.0)
        turn_left = left >= right
        turn_sign = 1.0 if turn_left else -1.0
        x, y, _ = self.pose
        goal_bearing = math.atan2(goal_y - y, goal_x - x)
        detour_yaw = wrap(
            goal_bearing + turn_sign * math.radians(self.avoid_turn_deg))
        self.stop()
        self.get_logger().warn(
            f'{label}：前方 {front:.2f} m，左侧 {left:.2f} m，'
            f'右侧 {right:.2f} m，向{"左" if turn_left else "右"}'
            f'转 {self.avoid_turn_deg:.0f}°绕行')

        start = time.monotonic()
        while rclpy.ok():
            self.tick()
            if time.monotonic() - self.last_scan_time > 1.5:
                self.stop()
                self.get_logger().error('避障时 /scan 超过 1.5 秒没数据，停车')
                return False
            error = wrap(detour_yaw - self.pose[2])
            if abs(error) < self.yaw_tol:
                break
            if time.monotonic() - start > self.avoid_turn_timeout:
                self.stop()
                self.get_logger().error(
                    f'{label}：绕行转向超时，角度误差 '
                    f'{math.degrees(error):+.1f}°')
                return False
            angular = clamp(
                1.7 * error, -self.avoid_turn_speed, self.avoid_turn_speed)
            if abs(angular) < 0.12:
                angular = 0.12 if error > 0.0 else -0.12
            self.publish(0.0, angular)

        self.stop()
        start = time.monotonic()
        start_x, start_y, detour_yaw = self.pose
        traveled = 0.0
        while rclpy.ok():
            self.tick()
            if time.monotonic() - self.last_scan_time > 1.5:
                self.stop()
                return False
            if self.front_range() < self.front_stop:
                break
            x, y, yaw = self.pose
            traveled = math.hypot(x - start_x, y - start_y)
            if traveled >= self.avoid_forward_distance:
                break
            if time.monotonic() - start > self.avoid_forward_timeout:
                break
            error = wrap(detour_yaw - yaw)
            angular = clamp(
                1.9 * error, -self.max_angular, self.max_angular)
            self.publish(self.avoid_forward_speed, angular)
        self.stop()
        self.get_logger().info(
            f'{label}：避让后沿新方向前进 {traveled:.2f} m')
        return self.front_range() >= self.front_stop

    # ---------- 控制 ----------
    def rotate_to(self, target_yaw, label):
        start = time.monotonic()
        while rclpy.ok():
            self.tick()
            error = wrap(target_yaw - self.pose[2])
            if abs(error) < self.yaw_tol:
                self.stop()
                return True
            if time.monotonic() - start > 15.0:
                self.stop()
                self.get_logger().error(f'{label}：转向超时')
                return False
            angular = clamp(1.7 * error, -self.max_angular, self.max_angular)
            if abs(angular) < 0.12:
                angular = 0.12 if error > 0.0 else -0.12
            self.publish(0.0, angular)
        self.stop()
        return False

    def drive_to(self, goal_x, goal_y, label):
        start = time.monotonic()
        blocked_since = None
        avoid_tries = 0
        prev_front = None
        prev_front_time = 0.0
        while rclpy.ok():
            self.tick()
            now = time.monotonic()
            x, y, yaw = self.pose
            distance = math.hypot(goal_x - x, goal_y - y)
            if distance < self.pos_tol:
                self.stop()
                self.get_logger().info(f'{label}：到位，误差 {distance:.3f} m')
                return True
            if now - start > self.leg_timeout:
                self.stop()
                self.get_logger().error(f'{label}：直行超时')
                return False

            if now - self.last_scan_time > 1.5:
                self.stop()
                self.get_logger().error('/scan 超过 1.5 秒没数据，停车')
                return False

            front = self.front_range()

            # 动态目标：前方距离在缩小（扣掉自身前进速度）就停车让行
            if self.last_linear >= 0.05 and front < self.dynamic_near:
                if prev_front is not None and now > prev_front_time:
                    closing = (prev_front - front) / (now - prev_front_time)
                    closing -= self.last_linear
                    if closing > self.dynamic_closing:
                        self.get_logger().warn(
                            f'{label}：前方目标在靠近（{closing:.2f} m/s）')
                        prev_front = None
                        if not self.wait_for_traffic(label, front):
                            return False
                        continue
            prev_front = front
            prev_front_time = now

            if front < self.front_stop:
                self.stop()
                if not self.avoidance:
                    if blocked_since is None:
                        blocked_since = now
                        self.get_logger().warn(
                            f'{label}：前方 {front:.2f} m 有障碍，停车等待')
                    elif now - blocked_since > self.obstacle_wait:
                        self.get_logger().error(f'{label}：障碍一直没让开，放弃')
                        return False
                    continue

                left = self.sector_range(45.0, 135.0)
                right = self.sector_range(-135.0, -45.0)
                if (self.side_override <= 0
                        and left < self.side_block
                        and right < self.side_block):
                    outcome = self.wait_side_blocked(
                        label, front, left, right)
                    if outcome is False:
                        return False
                    if outcome == 'manual':
                        self.side_override = 1
                    continue
                if self.side_override > 0:
                    self.side_override -= 1
                    self.get_logger().warn(
                        f'{label}：人工放行，跳过侧向检查，强制绕行一次')

                if avoid_tries >= self.avoid_max_tries:
                    self.get_logger().error(
                        f'{label}：连续避障 {avoid_tries} 次仍无法通过')
                    return False
                avoid_tries += 1
                if not self.avoid_obstacle(label, front, goal_x, goal_y):
                    return False
                blocked_since = None
                continue
            blocked_since = None

            heading = math.atan2(goal_y - y, goal_x - x)
            error = wrap(heading - yaw)
            angular = clamp(1.9 * error, -self.max_angular, self.max_angular)
            linear = 0.0 if abs(error) > 0.55 else clamp(
                0.9 * distance, 0.045, self.max_linear)
            if front < self.front_slow:
                linear = min(linear, 0.06)
            if distance < 0.14:
                linear = min(linear, 0.05)

            self.last_linear = linear
            self.publish(linear, angular)
            if now - self.last_log_time > 1.0:
                self.last_log_time = now
                self.get_logger().info(
                    f'{label}：剩余 {distance:.2f} m，'
                    f'航向误差 {math.degrees(error):+.1f}°，'
                    f'前方 {front:.2f} m')
        self.stop()
        return False

    def goto(self, goal_x, goal_y, goal_yaw, label):
        x, y, yaw = self.pose
        distance = math.hypot(goal_x - x, goal_y - y)
        self.get_logger().info(
            f'{label}：目标 ({goal_x:.2f}, {goal_y:.2f}, '
            f'{math.degrees(goal_yaw):.0f}°)，距离 {distance:.2f} m')

        if distance > 0.12:
            if not self.rotate_to(math.atan2(goal_y - y, goal_x - x), label):
                return False
            if not self.drive_to(goal_x, goal_y, label):
                return False

        if abs(wrap(goal_yaw - self.pose[2])) > self.yaw_tol:
            if not self.rotate_to(goal_yaw, label):
                return False
        return True

    def pause_for_operator(self):
        self.stop()
        self.get_logger().info('已到巡检点，停车')
        self.run_arm_demo()
        self.get_logger().info('开始遥操作；完成后按回车返航')
        done = threading.Event()

        def wait_input():
            try:
                input()
            except EOFError:
                pass
            done.set()

        threading.Thread(target=wait_input, daemon=True).start()
        while not done.is_set() and rclpy.ok():
            self.publish(0.0, 0.0)
            self.tick()
        self.stop()

    def run_arm_demo(self):
        if not self.arm_demo:
            return
        if not os.path.isfile(self.arm_demo_script):
            self.get_logger().error(
                f'找不到机械臂示教脚本：{self.arm_demo_script}')
            return

        command = [
            sys.executable,
            self.arm_demo_script,
            '--time-ms', str(max(200, self.arm_demo_time_ms)),
        ]
        if self.arm_demo_poses:
            command.extend(['--poses', self.arm_demo_poses])

        self.get_logger().info('执行机械臂示教动作，车辆保持停车')
        try:
            process = subprocess.Popen(command)
        except OSError as exc:
            self.get_logger().error(f'机械臂示教动作启动失败：{exc}')
            return

        while process.poll() is None and rclpy.ok():
            self.publish(0.0, 0.0)
            self.tick()

        code = process.returncode
        if code == 0:
            self.get_logger().info('机械臂示教动作完成')
        else:
            self.get_logger().error(f'机械臂示教动作失败，退出代码 {code}')

    # ---------- 主流程 ----------
    def run(self):
        if not self.wait_for_odom():
            return 1
        if not self.wait_for_scan():
            return 1

        start_x, start_y, start_yaw = self.pose
        self.get_logger().info(
            f'起点 odom=({start_x:.2f}, {start_y:.2f}, '
            f'{math.degrees(start_yaw):.0f}°)')

        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)
        target_x = (start_x + cos_yaw * self.target[0]
                    - sin_yaw * self.target[1])
        target_y = (start_y + sin_yaw * self.target[0]
                    + cos_yaw * self.target[1])
        target_yaw = start_yaw + self.target[2]

        if not self.goto(target_x, target_y, target_yaw, '去巡检点'):
            return 1

        if self.do_pause:
            self.pause_for_operator()

        if not self.return_home:
            return 0

        home_x = (start_x + cos_yaw * self.home[0]
                  - sin_yaw * self.home[1])
        home_y = (start_y + sin_yaw * self.home[0]
                  + cos_yaw * self.home[1])
        home_yaw = start_yaw + self.home[2]
        if not self.goto(home_x, home_y, home_yaw, '返航'):
            return 1
        return 0


def main():
    rclpy.init()
    node = OdomMission()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        print('\n中断。')
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
