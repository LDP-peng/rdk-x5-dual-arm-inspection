import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PatrolManager(Node):
    IDLE = 'IDLE'
    PATROL = 'PATROL'
    ALERT = 'ALERT'
    RETURN = 'RETURN'
    ERROR = 'ERROR'

    def __init__(self):
        super().__init__('patrol_manager')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('alarm_topic', '/vision/fire_smoke/alarm')
        self.declare_parameter('state_topic', '/mission/state')
        self.declare_parameter('alert_hold_sec', 5.0)
        self.declare_parameter('return_on_alert', True)
        self.declare_parameter('loop', True)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('goal_timeout_sec', 60.0)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.alarm_topic = self.get_parameter('alarm_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.alert_hold_sec = float(self.get_parameter('alert_hold_sec').value)
        self.return_on_alert = bool(self.get_parameter('return_on_alert').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)

        self.frame_id = 'map'
        self.points = []
        self._load_waypoints()

        self.state = self.IDLE
        self.current_index = 0
        self.current_goal_handle = None
        self._waiting_result = False
        self._last_goal_time = 0.0
        self._alert_timer = None
        self._started = False

        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.alarm_sub = self.create_subscription(
            Bool, self.alarm_topic, self.alarm_callback, 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info(
            f'patrol_manager started: waypoints={len(self.points)}, '
            f'frame_id={self.frame_id}, alarm_topic={self.alarm_topic}'
        )

    def _load_waypoints(self):
        path = self.waypoints_file
        if not path or not os.path.exists(path):
            self.get_logger().error(f'waypoints file not found: {path}')
            return

        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        self.frame_id = data.get('frame_id', 'map')
        self.points = data.get('points', [])
        self.get_logger().info(f'loaded {len(self.points)} waypoints from {path}')

    def tick(self):
        if self.state == self.IDLE and self.auto_start and not self._started:
            if self.points and self.nav_client.server_is_ready():
                self._started = True
                self.start_patrol()
            return

        if (
            self.state == self.PATROL
            and self._waiting_result
            and time.time() - self._last_goal_time > self.goal_timeout_sec
        ):
            self.get_logger().error('navigation goal timeout')
            self.cancel_current_goal()
            self.set_state(self.ERROR)

    def start_patrol(self):
        self.current_index = 0
        self.set_state(self.PATROL)
        self.send_current_goal()

    def send_current_goal(self):
        if not self.points:
            self.set_state(self.IDLE)
            return

        if self.current_index >= len(self.points):
            self.current_index = 0

        point = self.points[self.current_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point.get('x', 0.0))
        goal.pose.pose.position.y = float(point.get('y', 0.0))
        goal.pose.pose.position.z = 0.0

        yaw = float(point.get('yaw', 0.0))
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.get_logger().info(
            f'sending goal {self.current_index}: '
            f'{point.get("name", "unnamed")} '
            f'({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f})'
        )
        self._waiting_result = True
        self._last_goal_time = time.time()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('navigation goal rejected')
            self._waiting_result = False
            self.set_state(self.ERROR)
            return

        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self._waiting_result = False
        self.current_goal_handle = None

        if self.state in (self.ALERT, self.ERROR):
            return

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'goal {self.current_index} reached')
            if self.state == self.RETURN:
                self.set_state(self.IDLE)
                return

            self.current_index += 1
            if self.current_index >= len(self.points):
                if self.loop:
                    self.current_index = 0
                else:
                    self.set_state(self.IDLE)
                    return
            self.send_current_goal()
            return

        self.get_logger().warn(f'navigation failed with status {status}')
        self.set_state(self.ERROR)

    def alarm_callback(self, msg):
        if not msg.data:
            return
        if self.state != self.PATROL:
            return

        self.get_logger().warn('fire/smoke alarm: stopping current patrol')
        self.set_state(self.ALERT)
        self.cancel_current_goal()

        if self._alert_timer is not None:
            self.destroy_timer(self._alert_timer)
        self._alert_timer = self.create_timer(
            self.alert_hold_sec, self.resume_after_alert
        )

    def resume_after_alert(self):
        if self._alert_timer is not None:
            self._alert_timer.cancel()
            self.destroy_timer(self._alert_timer)
            self._alert_timer = None

        if self.return_on_alert:
            self.get_logger().info('alert hold finished: returning home')
            self.set_state(self.RETURN)
            self.current_index = 0
            self.send_current_goal()
        else:
            self.get_logger().info('alert hold finished: resuming patrol')
            self.set_state(self.PATROL)
            self.send_current_goal()

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self._waiting_result = False

    def set_state(self, state):
        if state == self.state:
            self.publish_state()
            return
        self.get_logger().info(f'mission state: {self.state} -> {state}')
        self.state = state
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()