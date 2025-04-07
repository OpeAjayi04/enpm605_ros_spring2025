
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange
from sensor_msgs.msg import Image, LaserScan
from radar_msgs.msg import RadarScan
from parameters_demo.utils import Color
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ProcessingDemoNode(Node):
    """
    A ROS2 node that processes data from multiple sensor sources.
    
    This node demonstrates advanced ROS2 concepts including:
    - Parameter handling with dynamic configuration
    - Multithreaded execution with callback groups
    - Quality of Service (QoS) settings
    - Multiple subscription management
    - Conditional processing based on parameter values
    
    The node subscribes to camera, lidar, and radar data streams and processes
    them according to a configurable processing mode. This architecture can serve
    as a template for sensor fusion applications.
    
    Parameters:
        processing_mode (string): Controls which sensor data to process
                                 (all, radar_only, camera_only, lidar_only)
        processing_rate (integer): Processing frequency in Hz (range: 1-30Hz)
        
    Subscribers:
        /camera/image_color (sensor_msgs/Image): Camera image data
        /lidar/points (sensor_msgs/LaserScan): Lidar scan data
        /radar/tracks (radar_msgs/RadarScan): Radar detection data
    """
    
    def __init__(self):
        """
        Initialize the processing demo node with parameters, subscribers, and timers.
        
        Sets up callback groups for parallel processing, declares and retrieves
        parameters, configures QoS profiles, creates subscribers for each sensor type,
        and initializes the processing timer.
        """
        super().__init__("processing_demo")
        
        # Define callback groups for different sensor types
        self._sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self._processing_cb_group = MutuallyExclusiveCallbackGroup()
        
        # -------------------------------------
        # Declare all parameters for the node
        # -------------------------------------
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "processing_mode",
                    "all",
                    ParameterDescriptor(description="Processing mode (all, radar_only, camera_only, lidar_only)"),
                ),
                (
                    "processing_rate",
                    10,
                    ParameterDescriptor(
                        description="Processing rate in Hz",
                        integer_range=[
                            IntegerRange(from_value=1, to_value=30, step=1)
                        ],
                    ),
                ),
            ],
        )
        
        # -------------------------------------
        # Get Parameters
        # -------------------------------------
        self._processing_mode = self.get_parameter("processing_mode").get_parameter_value().string_value
        self._processing_rate = self.get_parameter("processing_rate").get_parameter_value().integer_value
        
        # Create QoS profile for better reliability in sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # -------------------------------------
        # Create subscribers for each sensor
        # -------------------------------------
        # Camera subscriber
        self._camera_subscriber = self.create_subscription(
            Image, 
            "/camera/image_color",
            self._camera_callback,
            sensor_qos,
            callback_group=self._sensor_cb_group
        )
        
        # Lidar subscriber
        self._lidar_subscriber = self.create_subscription(
            LaserScan,
            "/lidar/points",
            self._lidar_callback,
            sensor_qos,
            callback_group=self._sensor_cb_group
        )
        
        # Radar subscriber
        self._radar_subscriber = self.create_subscription(
            RadarScan,
            "/radar/tracks",
            self._radar_callback,
            sensor_qos,
            callback_group=self._sensor_cb_group
        )
        
        # -------------------------------------
        # Set up processing timer
        # -------------------------------------
        processing_period = 1.0 / self._processing_rate
        self._processing_timer = self.create_timer(
            processing_period, 
            self._process_data_callback,
            callback_group=self._processing_cb_group
        )
        
        # Data storage for sensor readings
        self._latest_camera_data = None
        self._latest_lidar_data = None
        self._latest_radar_data = None
        
        # Statistics counters
        self._camera_frames_received = 0
        self._lidar_scans_received = 0
        self._radar_scans_received = 0
        self._processing_cycles = 0
        
        self.get_logger().info(f"{Color.CYAN}Processing Demo Node initialized with mode: {self._processing_mode}{Color.RESET}")
        self.get_logger().info(f"{Color.CYAN}Processing rate: {self._processing_rate} Hz{Color.RESET}")
    
    def _camera_callback(self, msg):
        """
        Callback function for camera data.
        
        Stores the latest camera frame and updates statistics. Logs information
        at a reduced frequency to prevent console spam.
        
        Args:
            msg (sensor_msgs/Image): The received camera image
        """
        self._latest_camera_data = msg
        self._camera_frames_received += 1
        
        # Log at reduced frequency to avoid console spam
        if self._camera_frames_received % 10 == 0:
            self.get_logger().debug(
                f"{Color.PURPLE}Received camera frame #{self._camera_frames_received} "
                f"size: {msg.width}x{msg.height}{Color.RESET}"
            )
    
    def _lidar_callback(self, msg):
        """
        Callback function for lidar data.
        
        Stores the latest lidar scan and updates statistics. Logs information
        at a reduced frequency to prevent console spam.
        
        Args:
            msg (sensor_msgs/LaserScan): The received lidar scan
        """
        self._latest_lidar_data = msg
        self._lidar_scans_received += 1
        
        # Log at reduced frequency to avoid console spam
        if self._lidar_scans_received % 10 == 0:
            self.get_logger().debug(
                f"{Color.GREEN}Received lidar scan #{self._lidar_scans_received} "
                f"with {len(msg.ranges)} points{Color.RESET}"
            )
    
    def _radar_callback(self, msg):
        """
        Callback function for radar data.
        
        Stores the latest radar scan and updates statistics. Logs information
        at a reduced frequency to prevent console spam.
        
        Args:
            msg (radar_msgs/RadarScan): The received radar scan
        """
        self._latest_radar_data = msg
        self._radar_scans_received += 1
        
        # Log at reduced frequency to avoid console spam
        if self._radar_scans_received % 10 == 0:
            self.get_logger().debug(
                f"{Color.CYAN}Received radar scan #{self._radar_scans_received} "
                f"with {len(msg.returns)} returns{Color.RESET}"
            )
    
    def _process_data_callback(self):
        """
        Main processing function that processes data from sensors.
        
        This function is called periodically by the timer at the rate specified
        by the processing_rate parameter. It selectively processes sensor data
        based on the current processing_mode parameter.
        
        The actual data processing algorithm would be implemented here in a
        real application. Currently, it only tracks statistics and reports status.
        """
        self._processing_cycles += 1
        
        # Skip processing if data is not available based on mode
        if self._processing_mode == "all":
            if not all([self._latest_camera_data, self._latest_lidar_data, self._latest_radar_data]):
                return
        elif self._processing_mode == "camera_only" and not self._latest_camera_data:
            return
        elif self._processing_mode == "lidar_only" and not self._latest_lidar_data:
            return
        elif self._processing_mode == "radar_only" and not self._latest_radar_data:
            return
        
        # Perform data processing based on mode
        # This is where you would implement your sensor data processing algorithm
        
        # Print status every few cycles
        if self._processing_cycles % 5 == 0:
            self._print_processing_status()
    
    def _print_processing_status(self):
        """
        Print processing status with statistics based on the current processing mode.
        
        Formats and outputs status information that adapts to the current processing mode.
        Only shows statistics relevant to the active sensors being processed, making
        the output more focused and readable.
        """
        status_header = f"\n{Color.YELLOW}===== Processing Status ====={Color.RESET}\n"
        status_footer = f"{Color.YELLOW}=============================\n{Color.RESET}"
        
        # Common information for all modes
        status_info = f"{Color.RED}Processing cycle: {self._processing_cycles}{Color.RESET}\n"
        
        # Mode-specific information
        if self._processing_mode == "all":
            status_info += (
                f"{Color.PURPLE}Camera frames received: {self._camera_frames_received}{Color.RESET}\n"
                f"{Color.GREEN}Lidar scans received: {self._lidar_scans_received}{Color.RESET}\n"
                f"{Color.CYAN}Radar scans received: {self._radar_scans_received}{Color.RESET}\n"
            )
        elif self._processing_mode == "camera_only":
            status_info += f"{Color.PURPLE}Camera frames received: {self._camera_frames_received}{Color.RESET}\n"
        elif self._processing_mode == "lidar_only":
            status_info += f"{Color.GREEN}Lidar scans received: {self._lidar_scans_received}{Color.RESET}\n"
        elif self._processing_mode == "radar_only":
            status_info += f"{Color.CYAN}Radar scans received: {self._radar_scans_received}{Color.RESET}\n"
        
        # Print complete status with header and footer
        self.get_logger().info(status_header + status_info + status_footer)


def main(args=None):
    """
    Main function for the processing demo node.
    
    Initializes ROS2 communication, creates the node, and sets up a MultiThreadedExecutor
    for parallel processing. Handles proper cleanup on exit with comprehensive error 
    handling for graceful termination.
    
    Args:
        args: Command line arguments passed to ROS2
        
    Note:
        This implementation uses a MultiThreadedExecutor with 4 threads to enable
        parallel processing of callbacks, improving overall throughput and responsiveness.
    """
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        # Create the node
        node = ProcessingDemoNode()
        # Create a MultiThreadedExecutor with specified number of threads
        executor = MultiThreadedExecutor(num_threads=4)
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
