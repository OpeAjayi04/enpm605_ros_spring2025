#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rosbot_interfaces.msg import ColorData
from rosbot_interfaces.srv import GetGoal
from rosbot_interfaces.action import MoveToGoal
import math


class NavigationController(Node):
    def __init__(self):
        super().__init__("navigation_controller_demo")

        # Parameters
        self.declare_parameter("target_color", "red")
        self.declare_parameter("cooldown_seconds", 5.0)
        self.declare_parameter("distance_tolerance", 0.1)
        self.declare_parameter("angle_tolerance", 0.05)
        self.declare_parameter("navigation_timeout", 60.0)
        self.declare_parameter("min_confidence", 0.7)
        self.declare_parameter("expected_frame_id", "odom")
        self.declare_parameter("feedback_throttle", 2.0)

        self._target_color = self.get_parameter("target_color").value
        self._cooldown_duration = Duration(
            seconds=self.get_parameter("cooldown_seconds").value
        )
        self._distance_tolerance = self.get_parameter("distance_tolerance").value
        self._angle_tolerance = self.get_parameter("angle_tolerance").value
        self._navigation_timeout = self.get_parameter("navigation_timeout").value
        self._min_confidence = self.get_parameter("min_confidence").value
        self._expected_frame_id = self.get_parameter("expected_frame_id").value
        self._feedback_throttle = self.get_parameter("feedback_throttle").value

        self._last_navigation_time = self.get_clock().now()
        self._is_navigating = False
        self._last_feedback_time = self.get_clock().now()
        self._current_goal_handle = None
        self._navigation_timer = None

        self._callback_group = ReentrantCallbackGroup()

        self._color_sub = self.create_subscription(
            ColorData,
            "/color_beacon",
            self._color_callback,
            10,
            callback_group=self._callback_group,
        )

        self._goal_client = self.create_client(
            GetGoal, "/get_goal", callback_group=self._callback_group
        )

        self._nav_client = ActionClient(
            self, MoveToGoal, "move_to_goal", callback_group=self._callback_group
        )

        self._wait_for_servers()

        self.get_logger().info("NavigationController node initialized")
        self.get_logger().info(
            f"Target color: {self._target_color}, Min confidence: {self._min_confidence}"
        )

    def _wait_for_servers(self):
        self.get_logger().info("Waiting for GetGoal service...")
        while not self._goal_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for service")
                return
            self.get_logger().info("Still waiting for GetGoal service...")

        self.get_logger().info("Waiting for MoveToGoal action server...")
        while not self._nav_client.wait_for_server(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for action server")
                return
            self.get_logger().info("Still waiting for MoveToGoal action server...")

    def _color_callback(self, msg):
        if msg.color != self._target_color or self._is_navigating:
            return

        # Consider putting this back
        # if msg.confidence < self._min_confidence:
        #     self.get_logger().debug(f"Low confidence for {msg.color}: {msg.confidence}")
        #     return

        current_time = self.get_clock().now()
        if current_time - self._last_navigation_time > self._cooldown_duration:
            self.get_logger().info(
                f"Detected {msg.color} with confidence {msg.confidence}"
            )
            self._request_navigation(msg.color)

    def _request_navigation(self, color):
        self._is_navigating = True
        self.get_logger().info(f"Requesting goal for color: {color}")

        request = GetGoal.Request()
        request.color = color
        future = self._goal_client.call_async(request)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            response = future.result()
            if response.success and self._validate_goal_pose(response.goal_pose):
                self.get_logger().info(
                    f"Goal pose received in frame {response.goal_pose.header.frame_id}"
                )
                self._send_navigation_goal(response.goal_pose)
            else:
                self.get_logger().error(f"Failed to get valid goal: {response.message}")
                self._is_navigating = False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.get_logger().error(f"Future exception: {future.exception()}")
            self._is_navigating = False

    def _validate_goal_pose(self, pose):
        if pose.header.frame_id != self._expected_frame_id:
            self.get_logger().warn(f"Unexpected frame: {pose.header.frame_id}")

        components = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        if any(math.isnan(x) for x in components):
            self.get_logger().error("Goal pose contains NaN values")
            return False

        quat_norm = (
            pose.pose.orientation.x**2
            + pose.pose.orientation.y**2
            + pose.pose.orientation.z**2
            + pose.pose.orientation.w**2
        )
        if abs(quat_norm - 1.0) > 0.01:
            self.get_logger().warn(
                f"Non-normalized quaternion (norm={quat_norm:.3f}), normalizing"
            )
            norm = math.sqrt(quat_norm)
            pose.pose.orientation.x /= norm
            pose.pose.orientation.y /= norm
            pose.pose.orientation.z /= norm
            pose.pose.orientation.w /= norm

        return True

    def _send_navigation_goal(self, pose):
        goal_msg = MoveToGoal.Goal()
        goal_msg.target_pose = pose
        goal_msg.distance_tolerance = self._distance_tolerance
        goal_msg.angle_tolerance = self._angle_tolerance

        self.get_logger().info(
            f"Sending navigation goal: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
        )

        send_goal_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self._navigation_goal_response_callback)

    def _navigation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            self._is_navigating = False
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted by action server")

        # Start timeout timer only after goal is accepted
        self._navigation_timer = self.create_timer(
            self._navigation_timeout, self._navigation_timeout_callback
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result_callback)

    def _navigation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_time = self.get_clock().now()
        if (
            current_time - self._last_feedback_time
        ).nanoseconds / 1e9 >= self._feedback_throttle:
            self.get_logger().info(
                f"Progress: {feedback.distance_remaining:.2f}m left, {feedback.angle_remaining:.2f} rad"
            )
            self._last_feedback_time = current_time

    def _navigation_result_callback(self, future):
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(f"Navigation completed: {result.message}")
            else:
                self.get_logger().error(f"Navigation failed: {result.message}")
        except Exception as e:
            self.get_logger().error(f"Navigation result error: {e}")

        self._cleanup_after_navigation()

    def _navigation_timeout_callback(self):
        if self._is_navigating:
            self.get_logger().error(
                f"Navigation timed out after {self._navigation_timeout} seconds"
            )
            if getattr(self._current_goal_handle, "accepted", False):
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)

        self._cleanup_after_navigation()

    def _cancel_done_callback(self, future):
        try:
            result = future.result()
            if result.return_code == result.SUCCEEDED:
                self.get_logger().info("Navigation goal successfully canceled")
            else:
                self.get_logger().warn("Failed to cancel navigation goal")
        except Exception as e:
            self.get_logger().error(f"Cancel error: {e}")

    def _cleanup_after_navigation(self):
        if self._navigation_timer is not None:
            self._navigation_timer.cancel()
            self._navigation_timer = None

        self._is_navigating = False
        self._last_navigation_time = self.get_clock().now()
        self._current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down from keyboard interrupt")
    finally:
        if node._navigation_timer is not None:
            node._navigation_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
