import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rosbot_interfaces.msg import ColorData
from std_msgs.msg import Header
from geometry_msgs.msg import Point


class ColorBeacon(Node):
    def __init__(self):
        super().__init__("color_beacon_demo")

        # Parameters
        self.declare_parameter("red_lower_hsv", [0, 30, 30])
        self.declare_parameter("red_upper_hsv", [15, 255, 255])
        self.declare_parameter("red_lower_hsv2", [160, 30, 30])
        self.declare_parameter("red_upper_hsv2", [179, 255, 255])
        self.declare_parameter("min_detection_area", 30)
        self.declare_parameter("rotation_speed", 0.3)
        self.declare_parameter("rotation_timer_period", 0.1)

        self._red_lower_hsv = np.array(self.get_parameter("red_lower_hsv").value)
        self._red_upper_hsv = np.array(self.get_parameter("red_upper_hsv").value)
        self._red_lower_hsv2 = np.array(self.get_parameter("red_lower_hsv2").value)
        self._red_upper_hsv2 = np.array(self.get_parameter("red_upper_hsv2").value)
        self._min_detection_area = self.get_parameter("min_detection_area").value
        self._rotation_speed = self.get_parameter("rotation_speed").value
        self._rotation_timer_period = self.get_parameter("rotation_timer_period").value

        # Rotation control flags
        self._should_rotate = True
        self._was_rotating = True  # Track previous rotation state
        self._last_red_detection_time = self.get_clock().now()
        self._red_detection_timeout = 1.0  # seconds
        self._color_detected = False

        # CV Bridge
        self._bridge = CvBridge()

        # Publishers
        self._color_pub = self.create_publisher(ColorData, "/color_beacon", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribers
        self._image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self._image_callback, 10
        )

        # Timer for persistent rotation
        self._rotation_timer = self.create_timer(
            self._rotation_timer_period, self._rotation_timer_callback
        )

        self.get_logger().info("ColorBeacon node initialized")

    def _image_callback(self, msg):
        if self._color_detected:
            return
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image to detect red color
            red_detected, area, _, _ = self._detect_red(cv_image)

            if red_detected:
                self._color_detected = True
                # Create and publish ColorData message
                color_msg = ColorData()
                color_msg.header = Header()
                color_msg.header.stamp = self.get_clock().now().to_msg()
                color_msg.header.frame_id = (
                    msg.header.frame_id
                )  # Use the same frame as the image
                color_msg.color = "red"
                color_msg.confidence = min(
                    1.0, area / 10000
                )  # Scale confidence based on area
                color_msg.area = area
                self._color_pub.publish(color_msg)

                self.get_logger().info(
                    f"Red detected with confidence: {color_msg.confidence}", once=True
                )
                # Update detection time and set rotation flag to false
                self._last_red_detection_time = self.get_clock().now()
                self._should_rotate = False
            else:
                # Check if it's been long enough since the last detection
                current_time = self.get_clock().now()
                time_since_detection = (
                    current_time - self._last_red_detection_time
                ).nanoseconds / 1e9

                if time_since_detection > self._red_detection_timeout:
                    if not self._should_rotate:
                        self.get_logger().info("Red not detected, enabling rotation")
                    self._should_rotate = True

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def _detect_red(self, image):
        """
        Detect red color in the image
        Returns: (red_detected, area, center_x, center_y)
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create masks for both red ranges (red wraps around in HSV)
        mask1 = cv2.inRange(hsv, self._red_lower_hsv, self._red_upper_hsv)
        mask2 = cv2.inRange(hsv, self._red_lower_hsv2, self._red_upper_hsv2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Try pixel-based detection first (more reliable for small areas)
        red_pixels = np.sum(mask) / 255  # Count white pixels in mask

        if red_pixels > self._min_detection_area:
            # Find the centroid of all red pixels
            y_coords, x_coords = np.where(mask > 0)
            if len(y_coords) > 0:
                center_x = int(np.mean(x_coords))
                center_y = int(np.mean(y_coords))
                return True, int(red_pixels), center_x, center_y

        # Find contours as backup method
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > self._min_detection_area:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    return True, int(area), center_x, center_y

        return False, 0, 0, 0

    def _rotation_timer_callback(self):
        """Control robot rotation based on detection state"""
        if self._color_detected:
            return
        # Only send commands when rotation state changes
        if self._should_rotate != self._was_rotating:
            twist = Twist()

            if self._should_rotate:
                # Start rotation
                twist.angular.z = self._rotation_speed
                self.get_logger().info(
                    f"Starting rotation at {self._rotation_speed} rad/s"
                )
            else:
                # Stop rotation
                twist.angular.z = 0.0
                self.get_logger().info("Stopping rotation - red detected")

            self._cmd_vel_pub.publish(twist)
            self._was_rotating = self._should_rotate
        elif self._should_rotate:
            # Continue rotation
            twist = Twist()
            twist.angular.z = self._rotation_speed
            self._cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Send command to stop the robot (called only during shutdown)"""
        twist = Twist()
        self._cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped")


def main(args=None):
    rclpy.init(args=args)

    color_beacon = ColorBeacon()

    try:
        rclpy.spin(color_beacon)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        # Stop the robot when shutting down
        if "color_beacon" in locals():
            color_beacon.stop_robot()
            color_beacon.destroy_node()
        rclpy.shutdown()
