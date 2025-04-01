import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherDemoNode(Node):
    def __init__(self):
        super().__init__('publisher_demo')
        self._count = 0
        # initialize publisher object
        self._leia_publisher = self.create_publisher(String, "leia", 3)
        # initialize timer object for publishing periodically
        self._leia_timer = self.create_timer(0.5, self._leia_pub_callback)
        # initialize message object
        self._leia_msg = String()
        
    def _leia_pub_callback(self):
        # Fill out message field(s)
        self._leia_msg.data = f'{self._count}: Help me Obiwan-Kenobi, you are my only hope'
        self._leia_publisher.publish(self._leia_msg)
        self.get_logger().info(f"Published: {self._leia_msg.data}")
        self._count += 1
        
def main(args=None):
    rclpy.init(args=args)
    node = PublisherDemoNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Initialization error: {e}")
    finally:
        node.get_logger().error("After Ctrl-C")
        node.destroy_node()
        rclpy.shutdown()