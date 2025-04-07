import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from remapping_demo.utils import Color
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class CameraDemoNode(Node):
    """
    A ROS2 node that simulates a camera by publishing random image data.

    This node demonstrates node, topic, and parameter remapping. It publishes random
    image data to simulate a camera feed.

    Parameters:
        camera_name (string): Name identifier for the camera
        camera_rate (integer): Frame rate in Hz, constrained to range 10-80 Hz in 10 Hz increments

    Publishers:
        /camera/image_color (sensor_msgs/Image): Publishes simulated camera images
    """

    def __init__(self):
        """
        Initialize the camera demo node with parameters, publishers, and timers.

        Declares and retrieves parameters, sets up the image publisher, and
        initializes the timer for periodic image publication.
        """
        super().__init__("camera_demo")
        # -------------------------------------
        # Declare all parameters for the node
        # -------------------------------------
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "camera_name",
                    "camera",
                    ParameterDescriptor(description="Camera name"),
                ),
                (
                    "camera_frame",
                    "camera_id",
                    ParameterDescriptor(description="Name of the camera frame"),
                ),
                (
                    "camera_rate",
                    60,
                    ParameterDescriptor(
                        description="Camera frame rate in Hz",
                        integer_range=[
                            IntegerRange(from_value=10, to_value=60, step=1)
                        ],
                    ),
                ),
            ],
        )

        # -------------------------------------
        # Get Parameters
        # -------------------------------------
        # Get param camera_name and store it for later use
        self._camera_name = (
            self.get_parameter("camera_name").get_parameter_value().string_value
        )

        # Get param camera_rate and store it for later use
        self._camera_rate = (
            self.get_parameter("camera_rate").get_parameter_value().integer_value
        )

        # Get param camera_frame and store it for later use
        self._camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        # Register a parameter callback for parameter changes
        self.add_on_set_parameters_callback(self._parameter_update_cb)

        # Pre-generate a random image to reuse (or update less frequently)

        self._image_width = 160  # 1/4 of the original width
        self._image_height = 120  # 1/4 of the original height
        self._image_channels = 3
        self._image_step = self._image_width * self._image_channels
        self._image_data = np.random.randint(
            0,
            256,
            size=(self._image_height, self._image_width * self._image_channels),
            dtype=np.uint8,
        ).tobytes()

        # Set up a publisher to /data/camera
        self._data_camera_publisher = self.create_publisher(
            Image, "camera/image_color", 10
        )
        # Set up a timer to publish at regular intervals
        self._data_camera_timer = self.create_timer(
            1.0 / self._camera_rate, self._data_camera_pub_callback
        )  # 60 Hz
        self._frame_counter = 0
        self._data_camera_msg = Image()

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
            if param.name == "camera_name":
                if param.type_ == Parameter.Type.STRING:  # validation
                    success = True
                    self._camera_name = param.value  # modify the attribute
                    self.get_logger().info(
                        f"Updated camera_name to: {self._camera_name}"
                    )
            elif param.name == "camera_rate":
                if param.type_ == Parameter.Type.INTEGER:  # validation
                    success = True
                    self._camera_rate = param.value
                    self.get_logger().info(
                        f"Updated camera_rate to: {self._camera_rate}"
                    )
                    if (
                        hasattr(self, "_data_camera_timer")
                        and self._data_camera_timer is not None
                    ):
                        self._data_camera_timer.cancel()
                        self._data_camera_timer = self.create_timer(
                            1 / self._camera_rate, self._data_camera_pub_callback
                        )
                        self.get_logger().info(
                            f"Timer rated updated to: {self._camera_rate}"
                        )

        if not success:
            return SetParametersResult(
                successful=False, reason="Invalid parameter type or name"
            )

        return SetParametersResult(successful=success)

    def _data_camera_pub_callback(self):
        """
        Timer callback function that publishes a simulated camera image.

        Generates a random RGB image and publishes it on the /camera/image_color topic.
        The image metadata uses the camera parameters, and image data is randomly generated.

        Note:
            This method is called periodically by the timer initialized in __init__.
            The publishing frequency is independent of the camera_rate parameter.
        """
        # Set header information
        self._data_camera_msg.header.stamp = self.get_clock().now().to_msg()
        self._data_camera_msg.header.frame_id = self._camera_frame

        self._data_camera_msg.height = self._image_height
        self._data_camera_msg.width = self._image_width
        self._data_camera_msg.encoding = "rgb8"
        self._data_camera_msg.is_bigendian = False
        self._data_camera_msg.step = self._image_step

        self._data_camera_msg.data = self._image_data

        # Publish the message
        self._data_camera_publisher.publish(self._data_camera_msg)

        self.get_logger().info(
            f"{Color.PURPLE}Published random camera data from:{Color.RESET} {Color.RED}{self._camera_name}{Color.RESET}"
        )


        # # Only log every 10th or 20th frame
        # if self._frame_counter % 20 == 0:
        #     self.get_logger().info(
        #         f"{Color.PURPLE}Published frame #{self._frame_counter} from:{Color.RESET} {Color.RED}{self._camera_name}{Color.RESET}"
        #     )
        # self._frame_counter += 1


def main(args=None):
    """
    Main function for the camera demo node.

    Initializes ROS2 communication, creates the node, and handles proper cleanup
    on exit. Includes comprehensive error handling.

    Args:
        args: Command line arguments passed to ROS2
    """
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
        if "node" in locals() and rclpy.ok():
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()
