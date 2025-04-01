import rclpy
from rclpy.node import Node

class AdvancedNode(Node):
    def __init__(self):
        super().__init__('advanced_node')
        self.get_logger().info("Hello World")
        
def main(args=None):
    rclpy.init(args=args)
    node = AdvancedNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Initialization error: {e}")
    finally:
        node.get_logger().error("After Ctrl-C")
        node.destroy_node()
        rclpy.shutdown()