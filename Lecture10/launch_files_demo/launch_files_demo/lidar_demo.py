import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from launch_files_demo.utils import Color

class LidarDemoNode(Node):
    def __init__(self):
        super().__init__('lidar_demo_node')
        self._data_lidar_publisher = self.create_publisher(LaserScan, '/data/lidar', 10)
        # Set up a timer to publish at regular intervals
        self._data_lidar_timer = self.create_timer(0.1, self._data_lidar_pub_callback)  # 10 Hz
        self._data_lidar_msg = LaserScan()
        
    def _data_lidar_pub_callback(self):
        # Set header information
        self._data_lidar_msg.header.stamp = self.get_clock().now().to_msg()
        self._data_lidar_msg.header.frame_id = "laser_frame"
        
        # Set the angle and range parameters
        self._data_lidar_msg.angle_min = -math.pi
        self._data_lidar_msg.angle_max = math.pi
        self._data_lidar_msg.angle_increment = math.pi / 180  # 1 degree increments
        self._data_lidar_msg.time_increment = 0.0
        self._data_lidar_msg.scan_time = 0.1
        self._data_lidar_msg.range_min = 0.2
        self._data_lidar_msg.range_max = 10.0
        
        # Number of readings based on angle range
        num_readings = int((self._data_lidar_msg.angle_max - self._data_lidar_msg.angle_min) / 
                           self._data_lidar_msg.angle_increment)
        
        # Generate random ranges and intensities
        self._data_lidar_msg.ranges = list(np.random.uniform(
            self._data_lidar_msg.range_min, self._data_lidar_msg.range_max, num_readings))
        self._data_lidar_msg.intensities = list(np.random.uniform(0.0, 1.0, num_readings))
        
        # Publish the message
        self._data_lidar_publisher.publish(self._data_lidar_msg)
        self.get_logger().info(f'{Color.GREEN}Published lidar data{Color.RESET}')

def main(args=None):
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        
        # Create and initialize the node
        node = LidarDemoNode()
        
        # Spin the node to process callbacks
        rclpy.spin(node)
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