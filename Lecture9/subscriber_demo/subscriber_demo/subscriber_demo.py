import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberDemoNode(Node):
    def __init__(self):
        super().__init__('subscriber_demo')
        self._leia_subscriber = self.create_subscription(String, "leia", self._leia_sub_callback, 3)
        
        
    def _leia_sub_callback(self, msg: String):
        self.get_logger().info(f"Received: {msg.data}")

        
def main(args=None):
    rclpy.init(args=args)
    node = SubscriberDemoNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Initialization error: {e}")
    finally:
        node.get_logger().error("After Ctrl-C")
        node.destroy_node()
        rclpy.shutdown()