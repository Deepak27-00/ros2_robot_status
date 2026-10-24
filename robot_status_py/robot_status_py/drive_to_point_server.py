"""Action server that drives the turtlesim turtle to an (x, y) goal.

Demonstrates:
  * an action server        -> /drive_to_point  (robot_status_interfaces/action/DriveToPoint)
  * goal / cancel callbacks, feedback and results
  * subscribing (/turtle1/pose) and publishing (/turtle1/cmd_vel) from inside a long-running callback,
    which needs a MultiThreadedExecutor so the pose callback can run while the goal executes.
"""
import math

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from robot_status_interfaces.action import DriveToPoint

# turtlesim's window spans roughly 0 .. 11.09 on both axes.
WORLD_MIN = 0.0
WORLD_MAX = 11.0
CONTROL_RATE_HZ = 20.0


class DriveToPointServer(Node):

    def __init__(self):
        super().__init__('drive_to_point_server')

        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('linear_gain', 1.5)
        self.declare_parameter('angular_gain', 6.0)
        self.declare_parameter('max_linear_speed', 2.0)

        self._pose = None
        self._goal_active = False

        self._cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.create_subscription(Pose, 'turtle1/pose', self._on_pose, 10)

        self._action_server = ActionServer(
            self,
            DriveToPoint,
            'drive_to_point',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Drive-to-point action server ready')

    def _on_pose(self, msg):
        self._pose = msg

    def _on_goal(self, goal_request):
        x, y = goal_request.x, goal_request.y
        if not (WORLD_MIN <= x <= WORLD_MAX and WORLD_MIN <= y <= WORLD_MAX):
            self.get_logger().warn(f'Rejecting goal ({x:.2f}, {y:.2f}): outside the turtlesim world')
            return GoalResponse.REJECT
        if self._goal_active:
            self.get_logger().warn('Rejecting goal: already driving to another goal')
            return GoalResponse.REJECT
        # Mark busy here (not in _execute) so a second goal arriving before execution starts is rejected.
        self._goal_active = True
        self.get_logger().info(f'Accepted goal ({x:.2f}, {y:.2f})')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        rate = self.create_rate(CONTROL_RATE_HZ)
        try:
            return self._drive(goal_handle, rate)
        finally:
            self._stop()
            self.destroy_rate(rate)
            self._goal_active = False

    def _drive(self, goal_handle, rate):
        goal = goal_handle.request
        tolerance = self.get_parameter('distance_tolerance').value
        k_lin = self.get_parameter('linear_gain').value
        k_ang = self.get_parameter('angular_gain').value
        max_speed = self.get_parameter('max_linear_speed').value
        feedback = DriveToPoint.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(False, 'Goal canceled')

            if self._pose is None:
                self.get_logger().info('Waiting for turtle1/pose...', throttle_duration_sec=2.0)
                rate.sleep()
                continue

            dx = goal.x - self._pose.x
            dy = goal.y - self._pose.y
            distance = math.hypot(dx, dy)

            if distance < tolerance:
                goal_handle.succeed()
                return self._make_result(True, f'Reached goal (error {distance:.3f})')

            # Simple proportional controller: turn towards the goal, and only drive forward
            # when roughly facing it (cos of the heading error scales the forward speed).
            heading_error = math.atan2(dy, dx) - self._pose.theta
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            cmd = Twist()
            cmd.linear.x = min(k_lin * distance, max_speed) * max(0.0, math.cos(heading_error))
            cmd.angular.z = k_ang * heading_error
            self._cmd_pub.publish(cmd)

            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        goal_handle.abort()
        return self._make_result(False, 'Node shutting down')

    def _make_result(self, success, message):
        result = DriveToPoint.Result()
        result.success = success
        result.message = message
        if self._pose is not None:
            result.final_x = self._pose.x
            result.final_y = self._pose.y
        self.get_logger().info(message)
        return result

    def _stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DriveToPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
