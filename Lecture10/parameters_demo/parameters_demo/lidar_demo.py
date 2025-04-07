import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from parameters_demo.utils import Color
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class LidarDemoNode(Node):
    """
    A ROS2 node that simulates a LiDAR sensor by publishing random scan data.
    
    This node demonstrates parameter handling in ROS2, including parameter declaration,
    retrieval, and dynamic updates through parameter callbacks. It publishes random
    LaserScan data to simulate LiDAR sensor readings with configurable properties.
    
    Parameters:
        lidar_name (string): Name identifier for the LiDAR sensor
        lidar_rate (integer): Scan rate in Hz, constrained to range 20-100 Hz in 20 Hz increments
        
    Publishers:
        /lidar/points (sensor_msgs/LaserScan): Publishes simulated LiDAR scan data
    """
    
    def __init__(self):
        """
        Initialize the LiDAR demo node with parameters, publishers, and timers.
        
        Declares and retrieves parameters, sets up the LaserScan publisher, and
        initializes the timer for periodic scan data publication.
        """
        super().__init__('lidar_demo')
        # Declare all parameters for the node
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "lidar_name",
                    "lidar",
                    ParameterDescriptor(description="Lidar name"),
                ),
                (
                    "lidar_model",
                    "OS2",
                    ParameterDescriptor(description="Lidar model"),
                ),
                (
                    "lidar_rate",
                    20,
                    ParameterDescriptor(
                        description="Lidar frame rate in Hz",
                        integer_range=[
                            IntegerRange(from_value=20, to_value=100, step=20)
                        ],
                    ),
                ),
            ],
        )
        # -------------------------------------
        # Get Parameters
        # -------------------------------------
        # Get param lidar_name and store it for later use
        self._lidar_name = (
            self.get_parameter("lidar_name").get_parameter_value().string_value
        )

        # Get param lidar_rate and store it for later use
        self._lidar_rate = (
            self.get_parameter("lidar_rate").get_parameter_value().integer_value
        )
        
        # Get param lidar_model and store it for later use
        self._lidar_model = (
            self.get_parameter("lidar_model").get_parameter_value().string_value
        )
        
        # Register a parameter callback for parameter changes
        # self.add_on_set_parameters_callback(self._parameter_update_cb)
        
        self._data_lidar_publisher = self.create_publisher(LaserScan, '/lidar/points', 10)
        # Set up a timer to publish at regular intervals
        self._data_lidar_timer = self.create_timer(1/self._lidar_rate, self._data_lidar_pub_callback)  # 10 Hz
        self._data_lidar_msg = LaserScan()
    
    def _parameter_update_cb(self, params):
        """
        Callback function triggered when node parameters are changed.
        
        This method is called automatically when parameters are updated via ROS2 parameter
        services. It validates the incoming parameters and updates the node's internal state
        accordingly.
        
        Args:
            params (list): List of Parameter objects that are being set
            
        Returns:
            SetParametersResult: Object indicating success or failure of parameter update
            
        Note:
            Only parameters defined in this method will be dynamically updatable.
            Parameter type checking is performed for validation.
        """
        success = False
        for param in params:
            if param.name == "lidar_name":
                if param.type_ == Parameter.Type.STRING:  # validation
                    success = True
                    self._lidar_name = param.value  # modify the attribute
                    self.get_logger().info(f"Updated lidar_name to: {self._lidar_name}")
            elif param.name == "lidar_rate":
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self._lidar_rate = param.value
                    self.get_logger().info(f"Updated lidar_rate to: {self._lidar_rate}")
        
        if not success:
            return SetParametersResult(successful=False, reason="Invalid parameter type or name")
            
        return SetParametersResult(successful=success)
    
    def _data_lidar_pub_callback(self):
        """
        Timer callback function that publishes a simulated LiDAR scan.
        
        Generates a synthetic LaserScan message with random range values and publishes
        it on the /lidar/points topic. Configures the scan with appropriate angle ranges,
        increments, and distance values.
        
        The scan data simulates a 360-degree LiDAR with 1-degree angular resolution
        and random range values between the minimum and maximum range settings.
        
        Note:
            This method is called periodically by the timer initialized in __init__.
            The publishing frequency is fixed at 10Hz, independent of the lidar_rate parameter.
        """
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
        self.get_logger().info(f'{Color.GREEN}Published random lidar ({self._lidar_model}) scan from: {Color.RESET}{Color.RED}{self._lidar_name}{Color.RESET}')


def main(args=None):
    """
    Main function for the LiDAR demo node.
    
    Initializes ROS2 communication, creates the node, and handles proper cleanup
    on exit. Includes comprehensive error handling for graceful termination.
    
    Args:
        args: Command line arguments passed to ROS2
    """
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
