import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class CameraDemoNode(Node):
    def __init__(self):
        super().__init__('camera_demo_node')
        self._data_camera_publisher = self.create_publisher(Image, '/data/camera', 10)
        # Set up a timer to publish at regular intervals
        self._data_camera_timer = self.create_timer(1.0, self._data_camera_pub_callback)  # 1 Hz
        self._data_camera_msg = Image()
        
    def _data_camera_pub_callback(self):
        
        # Set header information
        self._data_camera_msg.header.stamp = self.get_clock().now().to_msg()
        self._data_camera_msg.header.frame_id = "camera_frame"
        
        # Set image parameters
        width = 640    # Image width
        height = 480   # Image height
        channels = 3   # RGB
        step = width * channels
        
        self._data_camera_msg.height = height
        self._data_camera_msg.width = width
        self._data_camera_msg.encoding = "rgb8"
        self._data_camera_msg.is_bigendian = False
        self._data_camera_msg.step = step
        
        # Generate random pixel data
        data = np.random.randint(0, 256, size=(height, step), dtype=np.uint8).tobytes()
        self._data_camera_msg.data = data
        
        # Publish the message
        self._data_camera_publisher.publish(self._data_camera_msg)
        self.get_logger().info('Published random camera image')

def main(args=None):
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        # Create and initialize the node
        node = CameraDemoNode()
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
