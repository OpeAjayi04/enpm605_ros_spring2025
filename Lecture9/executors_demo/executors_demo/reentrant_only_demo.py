from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from executors_demo.utils import Color



class ReentrantOnlyDemoNode(Node):
    """
    Class to demonstrate the use of a reentrant callback group.
    """
    def __init__(self, node_name):
        super().__init__(node_name)

        group = ReentrantCallbackGroup()

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback, callback_group=group)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback, callback_group=group)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback, callback_group=group)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback, callback_group=group)

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
        time.sleep(10)

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
    rclpy.init(args=args)
    node = ReentrantOnlyDemoNode("reentrant_only_demo")

    # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()