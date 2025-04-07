import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from radar_msgs.msg import RadarScan, RadarReturn
from std_msgs.msg import Header
import random
import numpy as np
from parameters_demo.utils import Color
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class RadarScanDemoNode(Node):
    """
    A ROS2 node that simulates an automotive radar sensor by publishing random radar scan data.
    
    This node demonstrates parameter handling in ROS2, including parameter declaration,
    retrieval, and dynamic updates through parameter callbacks. It publishes random RadarScan
    messages with multiple radar returns to simulate an automotive radar sensor.
    
    The node uses a multithreaded executor for improved performance and to demonstrate
    more advanced ROS2 concepts.
    
    Parameters:
        radar_name (string): Name identifier for the radar sensor
        radar_rate (integer): Scan rate in Hz, constrained to range 55-80 Hz in 1 Hz increments
        
    Publishers:
        /radar/tracks (radar_msgs/RadarScan): Publishes simulated radar scan data
    """
    
    def __init__(self):
        """
        Initialize the radar demo node with parameters, publishers, and timers.
        
        Declares and retrieves parameters, sets up the RadarScan publisher, and
        initializes the timer for periodic radar data publication.
        """
        super().__init__("radar_demo")
        # Declare all parameters for the node
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "radar_name",
                    "radar",
                    ParameterDescriptor(description="Radar name"),
                ),
                (
                    "radar_rate",
                    79,
                    ParameterDescriptor(
                        description="Radar frame rate in Hz",
                        integer_range=[
                            IntegerRange(from_value=55, to_value=80, step=1)
                        ],
                    ),
                ),
            ],
        )
        # -------------------------------------
        # Get Parameters
        # -------------------------------------
        # Get param radar_name and store it for later use
        self._radar_name = (
            self.get_parameter("radar_name").get_parameter_value().string_value
        )

        # Get param radar_rate and store it for later use
        self._radar_rate = (
            self.get_parameter("radar_rate").get_parameter_value().integer_value
        )

        # Register a parameter callback for parameter changes
        # self.add_on_set_parameters_callback(self._parameter_update_cb)

        self._radar_msg = RadarScan()
        # Create publisher for the radar scan topic
        self._radar_tracks_publisher = self.create_publisher(
            RadarScan,
            "/radar/tracks",
            10,  # QoS profile depth
        )

        # Set timer for periodic publishing
        timer_period = 1/self._radar_rate
        self._radar_tracks_timer = self.create_timer(
            timer_period, self._radar_tracks_pub_callback
        )

        self.get_logger().info("RadarScan publisher node initialized.")

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
            if param.name == "radar_name":
                if param.type_ == Parameter.Type.STRING:  # validation
                    success = True
                    self._radar_name = param.value  # modify the attribute
                    self.get_logger().info(f"Updated radar_name to: {self._radar_name}")
            elif param.name == "radar_rate":
                if param.type_ == Parameter.Type.INTEGER:
                    success = True
                    self._radar_rate = param.value
                    self.get_logger().info(f"Updated radar_rate to: {self._radar_rate}")

        if not success:
            return SetParametersResult(
                successful=False, reason="Invalid parameter type or name"
            )

        return SetParametersResult(successful=success)

    def _radar_tracks_pub_callback(self):
        """
        Timer callback function that publishes a simulated radar scan.
        
        Generates a synthetic RadarScan message with multiple random radar returns and 
        publishes it on the /radar/tracks topic. Each scan contains between 1-10 returns
        with randomized properties including range, azimuth, elevation, Doppler velocity,
        and amplitude.
        
        The radar returns simulate targets at various distances and angles within the
        radar's field of view, with random motion characteristics.
        
        Note:
            This method is called periodically by the timer initialized in __init__.
            The publishing frequency is fixed at 10Hz, independent of the radar_rate parameter.
        """
        # Set header with current time
        self._radar_msg.header = Header()
        self._radar_msg.header.stamp = self.get_clock().now().to_msg()
        self._radar_msg.header.frame_id = "radar_link"

        # Random number of returns (1-10)
        num_returns = random.randint(1, 10)

        # Generate random returns
        for i in range(num_returns):
            radar_return = RadarReturn()

            # Random range in meters (0-100m)
            radar_return.range = random.uniform(0.0, 100.0)

            # Random azimuth angle in radians (-π/2 to π/2)
            radar_return.azimuth = random.uniform(-np.pi / 2, np.pi / 2)

            # Random elevation angle in radians (-π/6 to π/6)
            radar_return.elevation = random.uniform(-np.pi / 6, np.pi / 6)

            # Random Doppler velocity between -50 and 50 m/s
            radar_return.doppler_velocity = random.uniform(-50.0, 50.0)
            # Random amplitude between 0 and 100
            radar_return.amplitude = random.uniform(0.0, 100.0)

            # Add the return to the message
            self._radar_msg.returns.append(radar_return)

        # Publish the message
        self._radar_tracks_publisher.publish(self._radar_msg)
        self.get_logger().info(
            f"{Color.YELLOW}Published random radar scan with {num_returns} returns{Color.RESET} from: {Color.RESET}{Color.RED}{self._radar_name}{Color.RESET}"
        )


def main(args=None):
    """
    Main function for the radar demo node.
    
    Initializes ROS2 communication, creates the node, and sets up a MultiThreadedExecutor
    for improved performance. Handles proper cleanup on exit with comprehensive error handling
    for graceful termination.
    
    Args:
        args: Command line arguments passed to ROS2
        
    Note:
        This node uses a MultiThreadedExecutor instead of the default single-threaded
        executor, allowing for better handling of multiple callbacks simultaneously.
    """
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        # Create the node
        node = RadarScanDemoNode()
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
        if "node" in locals() and rclpy.ok():
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()