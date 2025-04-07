import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Image, Temperature
from launch_files_demo.utils import Color

class ProcessingDemoNode(Node):
    def __init__(self):
        super().__init__('processing_demo_node')
        
        # Create subscribers for the three different data types
        self._data_lidar_subscription = self.create_subscription(
            LaserScan,
            '/data/lidar',
            self._data_lidar_callback,
            10)
            
        self._data_camera_subscription = self.create_subscription(
            Image,
            '/data/camera',
            self._data_camera_callback,
            10)
            
        self._data_temperature_subscription = self.create_subscription(
            Temperature,
            '/data/temperature',
            self._data_temperature_callback,
            10)
    
    def _data_lidar_callback(self, msg):
        self.get_logger().info(f"{Color.GREEN}Received LiDAR data: ranges[0]={msg.ranges[0]:.2f}{Color.RESET}")
    
    def _data_camera_callback(self, msg):
        self.get_logger().info(f"{Color.PURPLE}Received Camera data: width={msg.width}, height={msg.height}{Color.RESET}")
     
    def _data_temperature_callback(self, msg):
        self.get_logger().info(f"{Color.YELLOW}Received Temperature data: {msg.temperature:.2f}°C{Color.RESET}")

def main(args=None):
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        
        # Create the node
        node = ProcessingDemoNode()
        
        # Create a MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # Spin the executor
        try:
            executor.spin()
        finally:
            executor.shutdown()
            
    except KeyboardInterrupt:
        # Handle clean Ctrl+C exit gracefully
        print("Node stopped cleanly by user")
    except Exception as e:
        # Catch and report any other exceptions
        print(f"Error occurred: {e}")
    finally:
        # Cleanup resources properly
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()