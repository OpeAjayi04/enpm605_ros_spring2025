from rclpy.node import Node
import time
from rclpy.executors import SingleThreadedExecutor
import rclpy
from executors_demo.utils import Color

    
class SingleThreadedDemoNode(Node):
    """
    Class to demonstrate the use of a single-threaded executor.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        
        self._test = 0

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback)

        self.get_logger().info(f"{node_name} initialized")

    def timer1_callback(self):
        """
        Callback function for timer1
        """
        self.get_logger().info(Color.YELLOW + "Timer1 callback" + Color.RESET)

    def timer2_callback(self):
        """
        Callback function for timer2
        """
        self.get_logger().info(Color.BLUE + "Timer2 callback" + Color.RESET)
        # time.sleep(2)
        # while True:
        #     pass

    def timer3_callback(self):
        """
        Callback function for timer3
        """
        self.get_logger().info(Color.GREEN + "Timer3 callback" + Color.RESET)

    def timer4_callback(self):
        """
        Callback function for timer4
        """
        self.get_logger().info(Color.RED + "Timer4 callback" + Color.RESET)

def main(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)
    node = SingleThreadedDemoNode("single_threaded_demo")

    executor = SingleThreadedExecutor()  # Explicit executor
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()
