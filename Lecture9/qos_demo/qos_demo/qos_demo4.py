import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time

class LatchedPublisherNode(Node):
    def __init__(self):
        super().__init__('latched_publisher')
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._status_msg_msg = String()
        self._status_msg_pub = self.create_publisher(String, 'status_msg', qos)
     
        self._count = 0
        self._status_msg_msg.data = f"Robot initialized at t={self._count}"
        self._status_msg_pub.publish(self._status_msg_msg)
        self.get_logger().info(f"Latched Published: {self._status_msg_msg.data}")



class LateSubscriberNode(Node):
    def __init__(self):
        super().__init__('late_subscriber')
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self._status_msg_sub = self.create_subscription(String, 'status_msg', self._status_msg_callback, qos)

    def _status_msg_callback(self, msg):
        self.get_logger().info(f"Received (late join): {msg.data}")


def main_latched_publisher(args=None):
    rclpy.init(args=args)
    node = LatchedPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_late_subscriber(args=None):
    rclpy.init(args=args)
    node = LateSubscriberNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
