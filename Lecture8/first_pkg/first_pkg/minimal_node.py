import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('minimal_node')
    node.get_logger().info("Hello World")
    node.get_logger().warn("Hello World")
    node.get_logger().error("Hello World")
    node.get_logger().fatal("Hello World")
    rclpy.shutdown()