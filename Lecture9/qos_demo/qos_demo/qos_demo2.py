import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time

class OverflowPublisherNode(Node):
    def __init__(self):
        super().__init__('overflow_publisher')
        qos = QoSProfile(depth=3)
        self._status_msg_msg = String()
        self._status_msg_pub = self.create_publisher(String, 'status_msg', qos)
        self._status_msg_timer = self.create_timer(0.5, self._status_msg_timer_callback)
        self._count = 0

    def _status_msg_timer_callback(self):
        self._status_msg_msg.data = f"status {self._count}"
        self._status_msg_pub.publish(self._status_msg_msg)
        self.get_logger().info(f"Published: {self._status_msg_msg.data}")
        self._count += 1


class SlowSubscriberNode(Node):
    def __init__(self):
        super().__init__('slow_subscriber')
        qos = QoSProfile(depth=3)
        self._status_msg_sub = self.create_subscription(String, 'status_msg', self._status_msg_callback, qos)

    def _status_msg_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        time.sleep(1.7)  # Slow processing → overflow happens


def main_overflow_publisher(args=None):
    rclpy.init(args=args)
    node = OverflowPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_slow_subscriber(args=None):
    rclpy.init(args=args)
    node = SlowSubscriberNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
