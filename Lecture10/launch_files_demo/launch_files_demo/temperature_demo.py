import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import numpy as np
from launch_files_demo.utils import Color

class TemperatureDemoNode(Node):
    def __init__(self):
        super().__init__('temperature_demo_node')
        self._data_temperature_publisher = self.create_publisher(
            Temperature, '/data/temperature', 10)
        # Set up a timer to publish at regular intervals
        self._data_temperature_timer = self.create_timer(
            1.0, self._data_temperature_pub_callback)  # 1 Hz
        self._data_temperature_msg = Temperature()
        
    def _data_temperature_pub_callback(self):
        # Set header information
        self._data_temperature_msg.header.stamp = self.get_clock().now().to_msg()
        self._data_temperature_msg.header.frame_id = "temperature_frame"
        
        # Generate random temperature data between -10°C and 40°C
        self._data_temperature_msg.temperature = float(np.random.uniform(-10.0, 40.0))
        self._data_temperature_msg.variance = 0.5  # Variance in the measurement
        
        # Publish the message
        self._data_temperature_publisher.publish(self._data_temperature_msg)
        self.get_logger().info(f"{Color.YELLOW}Published temperature: {self._data_temperature_msg.temperature:.2f}°C{Color.RESET}")

def main(args=None):
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        # Create and initialize the node
        node = TemperatureDemoNode()
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