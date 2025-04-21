import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
from tf2_ros import TransformException
import numpy as np
import math
import tf_transformations
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation
from tf2_ros.buffer import Buffer


class ColorPoseLogger(Node):
    def __init__(self):
        super().__init__("color_pose_logger")

        # Parameters for various colors
        self.declare_parameter("target_color", "yellow")
        self.declare_parameter("red_lower_hsv", [0, 30, 30])
        self.declare_parameter("red_upper_hsv", [15, 255, 255])
        self.declare_parameter("red_lower_hsv2", [160, 30, 30])
        self.declare_parameter("red_upper_hsv2", [179, 255, 255])
        self.declare_parameter("blue_lower_hsv", [100, 30, 30])
        self.declare_parameter("blue_upper_hsv", [130, 255, 255])
        self.declare_parameter("green_lower_hsv", [40, 30, 30])
        self.declare_parameter("green_upper_hsv", [80, 255, 255])
        self.declare_parameter("yellow_lower_hsv", [20, 30, 30])
        self.declare_parameter("yellow_upper_hsv", [35, 255, 255])
        self.declare_parameter("min_detection_area", 30)

        # Parameters for 3D pose estimation
        self.declare_parameter("cube_size_meters", 0.1)
        self.declare_parameter("camera_focal_length_pixels", 500.0)
        self.declare_parameter("pose_smoothing_factor", 0.7)
        self.declare_parameter("depth_correction_factor", 1.0)
        self.declare_parameter("parent_frame", "camera_color_optical_frame")
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("child_frame", "detected_color_block")

        # Get target color
        self._target_color = self.get_parameter("target_color").value

        # Get all color thresholds
        self._red_lower_hsv = np.array(self.get_parameter("red_lower_hsv").value)
        self._red_upper_hsv = np.array(self.get_parameter("red_upper_hsv").value)
        self._red_lower_hsv2 = np.array(self.get_parameter("red_lower_hsv2").value)
        self._red_upper_hsv2 = np.array(self.get_parameter("red_upper_hsv2").value)
        self._blue_lower_hsv = np.array(self.get_parameter("blue_lower_hsv").value)
        self._blue_upper_hsv = np.array(self.get_parameter("blue_upper_hsv").value)
        self._green_lower_hsv = np.array(self.get_parameter("green_lower_hsv").value)
        self._green_upper_hsv = np.array(self.get_parameter("green_upper_hsv").value)
        self._yellow_lower_hsv = np.array(self.get_parameter("yellow_lower_hsv").value)
        self._yellow_upper_hsv = np.array(self.get_parameter("yellow_upper_hsv").value)

        self._min_detection_area = self.get_parameter("min_detection_area").value

        # Get 3D pose estimation parameters
        self._cube_size_meters = self.get_parameter("cube_size_meters").value
        self._camera_focal_length_pixels = self.get_parameter(
            "camera_focal_length_pixels"
        ).value
        self._pose_smoothing_factor = self.get_parameter("pose_smoothing_factor").value
        self._depth_correction_factor = self.get_parameter(
            "depth_correction_factor"
        ).value

        # Frame parameters
        self._global_frame = self.get_parameter("global_frame").value
        self._parent_frame = self.get_parameter("parent_frame").value
        self._child_frame = self.get_parameter("child_frame").value

        # Camera intrinsics (will be updated if we receive camera info)
        self._camera_matrix = np.array(
            [
                [self._camera_focal_length_pixels, 0, 0],
                [0, self._camera_focal_length_pixels, 0],
                [0, 0, 1],
            ]
        )
        self._dist_coeffs = np.zeros((5, 1))  # Default: no distortion
        self._camera_info_received = False

        # For pose filtering
        self._last_position = None
        self._last_quaternion = None
        self._last_valid_pose_time = None

        # CV Bridge
        self._bridge = CvBridge()

        # Subscribers
        self._image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self._image_callback, 10
        )

        # Subscribe to camera info to get accurate camera parameters
        self._camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._camera_info_callback, 10
        )

        self.get_logger().info(
            f"ColorPoseLogger node initialized, looking for {self._target_color} color"
        )

        # TF system setup
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # Flag to track if a transform has been broadcasted
        self._transform_broadcasted = False
        
        # Create timer to periodically check transforms
        self._listener_timer = self.create_timer(1.0, self._check_transform)

    def _camera_info_callback(self, msg):
        """Process camera info to extract intrinsic parameters"""
        if not self._camera_info_received:
            # Extract camera matrix from camera info
            self._camera_matrix = np.array(msg.k).reshape(3, 3)

            # Extract distortion coefficients if available
            if hasattr(msg, "distortion_model") and msg.distortion_model != "":
                self._dist_coeffs = np.array(msg.d)

            self._camera_info_received = True
            self.get_logger().info("Camera intrinsic parameters received")
            self.get_logger().debug(f"Camera matrix: {self._camera_matrix}")

    def _check_transform(self):
        """
        Check and log the transform from global frame to the detected color block.
        """
        if not self._transform_broadcasted:
            return
            
        try:
            # Look up transform from global frame to the color block
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract and log the pose
            position = transform.transform.translation
            quaternion = transform.transform.rotation
            
            # Convert quaternion to RPY for more readable output
            quat_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            euler = tf_transformations.euler_from_quaternion(quat_list)
            roll, pitch, yaw = [math.degrees(angle) for angle in euler]
            
            # Log the pose (throttled to avoid spam)
            now = self.get_clock().now()
            if (not hasattr(self, "_last_log_time") or 
                (now - self._last_log_time).nanoseconds / 1e9 > 1.0):
                pos_str = f"[{position.x:.3f}, {position.y:.3f}, {position.z:.3f}]"
                rpy_str = f"[{roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°]"
                self.get_logger().info(
                    f"{self._target_color.capitalize()} block in {self._global_frame} frame:\n"
                    f"Position: {pos_str}\n"
                    f"Orientation: {rpy_str}"
                )
                self._last_log_time = now
                
        except TransformException as e:
            # Only log errors occasionally
            now = self.get_clock().now()
            if (not hasattr(self, "_last_error_time") or 
                (now - self._last_error_time).nanoseconds / 1e9 > 5.0):
                self.get_logger().debug(f"Transform not available: {e}")
                self._last_error_time = now

    def _image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")

            # Get image dimensions
            height, width, _ = cv_image.shape

            # Update camera matrix center point if not set properly
            if not self._camera_info_received:
                # Estimate camera center as the image center
                self._camera_matrix[0, 2] = width / 2.0
                self._camera_matrix[1, 2] = height / 2.0

            # Detect the target color
            color_detected = False
            area = 0
            position_x = 0
            position_y = 0
            contour = None

            # Handle red detection (which often requires two HSV ranges)
            if self._target_color == "red":
                color_detected, area, position_x, position_y, contour = (
                    self._detect_color_with_multiple_ranges(
                        cv_image,
                        [self._red_lower_hsv, self._red_lower_hsv2],
                        [self._red_upper_hsv, self._red_upper_hsv2],
                    )
                )
            # Handle all other colors
            elif self._target_color in ["green", "blue", "yellow"]:
                if self._target_color == "green":
                    lower_hsv, upper_hsv = self._green_lower_hsv, self._green_upper_hsv
                elif self._target_color == "blue":
                    lower_hsv, upper_hsv = self._blue_lower_hsv, self._blue_upper_hsv
                elif self._target_color == "yellow":
                    lower_hsv, upper_hsv = self._yellow_lower_hsv, self._yellow_upper_hsv
                
                color_detected, area, position_x, position_y, contour = self._detect_color(
                    cv_image, lower_hsv, upper_hsv
                )
            else:
                self.get_logger().warn(
                    f"Unknown target color: {self._target_color}", once=True
                )
                return

            if color_detected:
                # Estimate 3D pose from 2D detection
                position_3d, quaternion = self._estimate_improved_pose(
                    contour, position_x, position_y, width, height
                )

                # Apply smoothing to reduce jitter
                position_3d, quaternion = self._smooth_pose(position_3d, quaternion)

                # Create and broadcast transform
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = self._parent_frame
                transform.child_frame_id = self._child_frame
                
                # Set translation
                transform.transform.translation.x = position_3d[0]
                transform.transform.translation.y = position_3d[1]
                transform.transform.translation.z = position_3d[2]
                
                # Set rotation
                transform.transform.rotation.x = quaternion[0]
                transform.transform.rotation.y = quaternion[1]
                transform.transform.rotation.z = quaternion[2]
                transform.transform.rotation.w = quaternion[3]
                
                # Broadcast the transform
                self._tf_broadcaster.sendTransform(transform)
                self._transform_broadcasted = True

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _detect_color_with_multiple_ranges(self, image, lower_hsv_list, upper_hsv_list):
        """
        Detect a color using multiple HSV ranges (useful for colors like red that wrap around the hue circle)
        Returns: (color_detected, area, center_x, center_y, contour)
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create a combined mask from all ranges
        combined_mask = np.zeros_like(hsv[:, :, 0])

        for lower_hsv, upper_hsv in zip(lower_hsv_list, upper_hsv_list):
            # Create mask for the current HSV range
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            # Combine with the previous masks
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Process the combined mask
        return self._process_color_mask(combined_mask, image)

    def _detect_color(self, image, lower_hsv, upper_hsv):
        """
        Detect a specific color in the image using the provided HSV bounds
        Returns: (color_detected, area, center_x, center_y, contour)
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for the color
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Process the mask
        return self._process_color_mask(mask, image)

    def _process_color_mask(self, mask, original_image):
        """
        Process a binary mask to determine if a color is detected and calculate orientation
        Returns: (color_detected, area, center_x, center_y, contour)
        """
        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours for orientation detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Try pixel-based detection first (more reliable for small areas)
        color_pixels = np.sum(mask) / 255  # Count white pixels in mask

        if color_pixels > self._min_detection_area:
            # Find the centroid of all colored pixels
            y_coords, x_coords = np.where(mask > 0)
            if len(y_coords) > 0:
                center_x = int(np.mean(x_coords))
                center_y = int(np.mean(y_coords))

                # If we have contours, use the largest one
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)

                    if area > self._min_detection_area:
                        # Apply contour approximation to reduce noise
                        epsilon = 0.02 * cv2.arcLength(largest_contour, True)
                        approx_contour = cv2.approxPolyDP(
                            largest_contour, epsilon, True
                        )

                        # Use the approximated contour if it has enough points
                        if len(approx_contour) >= 4:  # At least 4 points for a quadrilateral
                            return True, int(color_pixels), center_x, center_y, approx_contour

                        return True, int(color_pixels), center_x, center_y, largest_contour

                # No valid contour, but we have pixel data
                return True, int(color_pixels), center_x, center_y, None

        # Find contours as backup method
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > self._min_detection_area:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])

                    # Apply contour approximation to reduce noise
                    epsilon = 0.02 * cv2.arcLength(largest_contour, True)
                    approx_contour = cv2.approxPolyDP(largest_contour, epsilon, True)

                    # Use the approximated contour if it has enough points
                    if len(approx_contour) >= 4:  # At least 4 points for a quadrilateral
                        return True, int(area), center_x, center_y, approx_contour

                    return True, int(area), center_x, center_y, largest_contour

        return False, 0, 0, 0, None

    def _estimate_improved_pose(self, contour, pixel_x, pixel_y, img_width, img_height):
        """
        Improved 3D pose estimation using PnP solver when possible, with considerations for depth.
        Returns: (position_3d, quaternion)
        """
        # Default values
        default_position = (0.0, 0.0, 1.0)
        default_quaternion = (0.0, 0.0, 0.0, 1.0)  # Identity quaternion

        if contour is None or not self._camera_info_received:
            return default_position, default_quaternion

        # Get camera intrinsics
        fx = self._camera_matrix[0, 0]  # Focal length x
        fy = self._camera_matrix[1, 1]  # Focal length y
        cx = self._camera_matrix[0, 2]  # Principal point x
        cy = self._camera_matrix[1, 2]  # Principal point y

        # Try to use solvePnP if we have enough contour points
        if len(contour) >= 4:
            try:
                # Attempt to approximate the contour to find corners
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # We need at least 4 points for solvePnP
                if len(approx) >= 4:
                    # Order the points for consistency
                    image_points = self._order_points(approx.reshape(-1, 2).astype(np.float32))

                    # Define 3D model points for a square face on the cube (z=0 plane)
                    half_size = self._cube_size_meters / 2.0
                    model_points = np.array([
                        [-half_size, -half_size, 0.0],  # top-left
                        [half_size, -half_size, 0.0],   # top-right
                        [half_size, half_size, 0.0],    # bottom-right
                        [-half_size, half_size, 0.0]    # bottom-left
                    ], dtype=np.float32)

                    # Ensure we have exactly 4 points
                    if len(image_points) == 4:
                        success, rvec, tvec = cv2.solvePnP(
                            model_points,
                            image_points,
                            self._camera_matrix,
                            self._dist_coeffs,
                            flags=cv2.SOLVEPNP_ITERATIVE
                        )

                        if success:
                            # Convert rotation vector to quaternion
                            rot_mat, _ = cv2.Rodrigues(rvec)
                            r = Rotation.from_matrix(rot_mat)
                            quat = r.as_quat()  # returns x, y, z, w
                            quaternion = (quat[0], quat[1], quat[2], quat[3])

                            # Extract position from translation vector
                            position_3d = (tvec[0][0], tvec[1][0], tvec[2][0])

                            position_3d = (
                                position_3d[0],
                                position_3d[1],
                                position_3d[2] * self._depth_correction_factor
                            )

                            return position_3d, quaternion
                        else:
                            self.get_logger().debug("solvePnP failed.")
            except Exception as e:
                self.get_logger().debug(f"Error in solvePnP: {e}")

        # Fallback method if solvePnP fails or we don't have enough contour points
        M = cv2.moments(contour)
        if M["m00"] > 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])

            # Calculate depth (Z) using apparent size
            rect = cv2.minAreaRect(contour)
            (_, _), (width_px, height_px), _ = rect
            apparent_size_px = max(width_px, height_px)

            if apparent_size_px > 0:
                z = (self._cube_size_meters * fx) / apparent_size_px * self._depth_correction_factor
                x = (center_x - cx) * z / fx
                y = (center_y - cy) * z / fy

                # Orientation from rectangle angle (yaw only)
                angle_deg = rect[2]
                if angle_deg < -45:
                    angle_deg += 90
                angle_rad = math.radians(angle_deg)
                roll, pitch, yaw = 0.0, 0.0, angle_rad
                quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw, 'rxyz')
                return (x, y, z), quaternion
            else:
                self.get_logger().debug("Apparent size is zero, cannot estimate depth.")
                return default_position, default_quaternion
        else:
            self.get_logger().debug("Could not calculate moments for fallback pose estimation.")
            return default_position, default_quaternion

    def _order_points(self, pts):
        """
        Order points in top-left, top-right, bottom-right, bottom-left order.
        """
        rect = cv2.minAreaRect(pts.reshape(-1, 1, 2).astype(np.float32))
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Sort the points based on their y-coordinates
        y_sorted = box[np.argsort(box[:, 1]), :]

        # Take the top two and bottom two
        top = y_sorted[:2, :]
        bottom = y_sorted[2:, :]

        # Sort the top two based on x-coordinates (top-left will have smaller x)
        tl = top[np.argmin(top[:, 0])]
        tr = top[np.argmax(top[:, 0])]

        # Sort the bottom two based on x-coordinates (bottom-left will have smaller x)
        bl = bottom[np.argmin(bottom[:, 0])]
        br = bottom[np.argmax(bottom[:, 0])]

        return np.array([tl, tr, br, bl], dtype=np.float32)

    def _smooth_pose(self, current_position, current_quaternion):
        """
        Apply temporal filtering to smooth the pose estimates
        Returns: (smoothed_position, smoothed_quaternion)
        """
        current_time = self.get_clock().now()

        # Initialize with current values if this is the first detection
        if self._last_position is None or self._last_quaternion is None:
            self._last_position = current_position
            self._last_quaternion = current_quaternion
            self._last_valid_pose_time = current_time
            return current_position, current_quaternion

        # Check how long since the last valid pose
        time_delta = (current_time - self._last_valid_pose_time).nanoseconds / 1e9

        # Reset smoothing if it's been too long
        if time_delta > 1.0:  # Detection timeout
            self._last_position = current_position
            self._last_quaternion = current_quaternion
            self._last_valid_pose_time = current_time
            return current_position, current_quaternion

        # Update last valid pose time
        self._last_valid_pose_time = current_time

        # Apply exponential smoothing to position
        alpha = 1.0 - self._pose_smoothing_factor
        smoothed_position = (
            alpha * current_position[0] + self._pose_smoothing_factor * self._last_position[0],
            alpha * current_position[1] + self._pose_smoothing_factor * self._last_position[1],
            alpha * current_position[2] + self._pose_smoothing_factor * self._last_position[2],
        )

        # Apply spherical linear interpolation (SLERP) to quaternions
        smoothed_quaternion = tf_transformations.quaternion_slerp(
            current_quaternion, self._last_quaternion, self._pose_smoothing_factor
        )

        # Store for next iteration
        self._last_position = smoothed_position
        self._last_quaternion = smoothed_quaternion

        return smoothed_position, smoothed_quaternion


def main(args=None):
    rclpy.init(args=args)

    color_pose_logger = ColorPoseLogger()

    try:
        rclpy.spin(color_pose_logger)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        color_pose_logger.destroy_node()
        rclpy.shutdown()