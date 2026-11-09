"""ROS 2 driver for the WHEELTEC C30D differential base."""

import math
import os
import select
import termios
import threading
import time
import traceback

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from robot_base.c30d_protocol import (
    ProtocolError,
    build_command,
    decode_feedback,
    extract_frames,
)


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


def covariance(diagonal):
    if len(diagonal) == 3:
        size = 3
    elif len(diagonal) == 6:
        size = 6
    else:
        raise ValueError('covariance expects 3 or 6 diagonal values')

    result = [0.0] * (size * size)
    for index, value in enumerate(diagonal):
        result[index * (size + 1)] = float(value)
    return result


class SerialPort:
    def __init__(self, port, baud_rate):
        if baud_rate != 115200:
            raise ValueError('C30D driver currently supports 115200 baud only')

        self.port = port
        self.fd = None
        self.write_lock = threading.Lock()

    def open(self):
        with self.write_lock:
            self._close_locked()
            fd = os.open(
                self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
            )
            try:
                attributes = termios.tcgetattr(fd)
                attributes[0] = 0
                attributes[1] = 0
                attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
                attributes[3] = 0
                attributes[4] = termios.B115200
                attributes[5] = termios.B115200
                attributes[6][termios.VMIN] = 0
                attributes[6][termios.VTIME] = 0
                termios.tcsetattr(fd, termios.TCSANOW, attributes)
                termios.tcflush(fd, termios.TCIFLUSH)
            except Exception:
                os.close(fd)
                raise
            self.fd = fd

    @property
    def is_open(self):
        return self.fd is not None

    def fileno(self):
        return self.fd

    def read(self, size):
        if self.fd is None:
            return b''
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b''

    def write(self, data):
        with self.write_lock:
            if self.fd is None:
                return False
            try:
                os.write(self.fd, data)
            except BlockingIOError:
                return False
            except OSError:
                self._close_locked()
                raise
        return True

    def close(self):
        with self.write_lock:
            self._close_locked()

    def _close_locked(self):
        if self.fd is None:
            return
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = None


class C30dDriver(Node):
    def __init__(self):
        super().__init__('c30d_driver')

        self.declare_parameter('port', '/dev/wheeltec_controller')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.30)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('accel_lsb_per_g', 16384.0)
        self.declare_parameter('gyro_lsb_per_dps', 16.4)

        self.port = str(self.get_parameter('port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)
        self.command_rate = float(self.get_parameter('command_rate').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.invert_linear = bool(self.get_parameter('invert_linear').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)
        self.accel_scale = 9.80665 / float(
            self.get_parameter('accel_lsb_per_g').value
        )
        self.gyro_scale = math.pi / (
            180.0 * float(self.get_parameter('gyro_lsb_per_dps').value)
        )

        self.serial = SerialPort(self.port, self.baud_rate)
        self.link_down = threading.Event()
        self.link_down.set()
        self.last_open_attempt = 0.0
        self.last_link_log = 0.0
        self.open_failures = 0
        self.last_stale_log = 0.0
        self.last_battery_log = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )

        self.state_lock = threading.Lock()
        self.latest_feedback = None
        self.latest_feedback_time = 0.0
        self.serial_buffer = bytearray()
        self.running = threading.Event()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)

        self.target_vx = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = time.monotonic() - self.cmd_timeout
        self.last_write_error = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = None

        self.command_timer = self.create_timer(
            1.0 / self.command_rate, self.send_command
        )
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_state
        )

        self.open_link()
        self.running.set()
        self.reader.start()
        self.get_logger().info(
            f'C30D base started on {self.port} at {self.baud_rate} baud'
        )

    def cmd_vel_callback(self, msg):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        if not math.isfinite(vx) or not math.isfinite(wz):
            return

        if self.invert_linear:
            vx = -vx
        if self.invert_angular:
            wz = -wz

        self.target_vx = clamp(vx, -self.max_linear_speed, self.max_linear_speed)
        self.target_wz = clamp(wz, -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_time = time.monotonic()

    def send_command(self):
        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.target_vx = 0.0
            self.target_wz = 0.0

        if self.link_down.is_set():
            return

        frame = build_command(
            int(round(self.target_vx * 1000.0)),
            int(round(self.target_wz * 1000.0)),
        )
        try:
            written = self.serial.write(frame)
        except OSError as exc:
            self.mark_link_down(f'write failed: {exc}')
            return
        if not written:
            now = time.monotonic()
            if now - self.last_write_error > 2.0:
                self.last_write_error = now
                self.get_logger().error('failed to write C30D command frame')

    def open_link(self, verbose=True):
        self.last_open_attempt = time.monotonic()
        try:
            self.serial.open()
        except OSError as exc:
            self.link_down.set()
            self.open_failures += 1
            now = time.monotonic()
            if verbose or now - self.last_link_log > 5.0:
                self.last_link_log = now
                self.get_logger().error(
                    f'cannot open {self.port}: {exc}; retrying every 1s'
                )
            return False

        self.serial_buffer.clear()
        with self.state_lock:
            self.latest_feedback = None
            self.latest_feedback_time = 0.0
        self.link_down.clear()
        if self.open_failures > 0:
            self.get_logger().warn(
                f'C30D link restored on {self.port} after '
                f'{self.open_failures} failed attempt(s)'
            )
        self.open_failures = 0
        return True

    def mark_link_down(self, reason):
        if not self.link_down.is_set():
            self.link_down.set()
            self.get_logger().error(
                f'C30D link lost: {reason}; reconnecting'
            )
        self.serial.close()

    def read_loop(self):
        while self.running.is_set() and rclpy.ok():
            if self.link_down.is_set():
                if time.monotonic() - self.last_open_attempt >= 1.0:
                    self.open_link(verbose=False)
                time.sleep(0.05)
                continue

            try:
                ready, _, _ = select.select([self.serial.fileno()], [], [], 0.1)
            except (OSError, ValueError) as exc:
                self.mark_link_down(f'select failed: {exc}')
                continue
            if not ready:
                continue

            try:
                data = self.serial.read(256)
            except OSError as exc:
                self.mark_link_down(f'read failed: {exc}')
                continue
            if not data:
                continue

            self.serial_buffer.extend(data)
            if len(self.serial_buffer) > 4096:
                del self.serial_buffer[:-2048]

            for frame in extract_frames(self.serial_buffer):
                try:
                    feedback = decode_feedback(frame)
                except ProtocolError:
                    continue
                with self.state_lock:
                    self.latest_feedback = feedback
                    self.latest_feedback_time = time.monotonic()

    def publish_state(self):
        with self.state_lock:
            feedback = self.latest_feedback
            feedback_time = self.latest_feedback_time

        if feedback is None:
            return

        now_monotonic = time.monotonic()
        stale = now_monotonic - feedback_time > 0.5
        if stale and self.last_odom_time is None:
            return

        now = self.get_clock().now()
        dt = 0.0 if self.last_odom_time is None else now_monotonic - self.last_odom_time
        if dt < 0.0 or dt > 0.5:
            dt = 0.0

        if stale:
            vx = 0.0
            wz = 0.0
            if now_monotonic - self.last_stale_log > 5.0:
                self.last_stale_log = now_monotonic
                self.get_logger().warn(
                    'C30D feedback stale, publishing frozen odom at zero velocity'
                )
        else:
            vx = feedback.vx_mm_s / 1000.0
            wz = feedback.wz_mrad_s / 1000.0
            if dt > 0.0:
                self.yaw += wz * dt
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
                self.x += vx * math.cos(self.yaw) * dt
                self.y += vx * math.sin(self.yaw) * dt
            self.last_odom_time = now_monotonic

            if now_monotonic - self.last_battery_log > 10.0:
                self.last_battery_log = now_monotonic
                volts = feedback.voltage_mv / 1000.0
                if 5.0 < volts < 11.0:
                    self.get_logger().warn(f'C30D battery low: {volts:.2f} V')
                elif volts >= 11.0:
                    self.get_logger().info(f'C30D battery: {volts:.2f} V')

        quaternion = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quaternion
        odom.pose.covariance = covariance((0.02, 0.02, 99999.0, 99999.0, 99999.0, 0.05))
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = covariance((0.005, 0.005, 0.005, 0.005, 0.005, 0.01))
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = quaternion
        self.tf_broadcaster.sendTransform(transform)

        if not stale:
            imu = Imu()
            imu.header.stamp = odom.header.stamp
            imu.header.frame_id = self.imu_frame
            imu.orientation = quaternion
            imu.orientation_covariance = covariance((0.10, 0.10, 0.10))
            imu.angular_velocity.x = feedback.gx_raw * self.gyro_scale
            imu.angular_velocity.y = feedback.gy_raw * self.gyro_scale
            imu.angular_velocity.z = feedback.gz_raw * self.gyro_scale
            imu.angular_velocity_covariance = covariance((0.01, 0.01, 0.01))
            imu.linear_acceleration.x = feedback.ax_raw * self.accel_scale
            imu.linear_acceleration.y = feedback.ay_raw * self.accel_scale
            imu.linear_acceleration.z = feedback.az_raw * self.accel_scale
            imu.linear_acceleration_covariance = covariance((0.05, 0.05, 0.05))
            self.imu_pub.publish(imu)

    def destroy_node(self):
        self.running.clear()
        if self.reader.is_alive():
            self.reader.join(timeout=1.0)
        self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = C30dDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
