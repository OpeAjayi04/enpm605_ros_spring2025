import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # Declare and get parameters
        self.declare_parameter("camera_raw_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter(
            "marker_size", 0.194
        )  # Size of the ArUco marker in meters
        self.declare_parameter("dictionary_id", "DICT_5X5_250")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("base_frame", "odom")  # Add base_frame parameter

        self._camera_raw_topic = self.get_parameter("camera_raw_topic").value
        self._camera_info_topic = self.get_parameter("camera_info_topic").value
        self._marker_size = self.get_parameter("marker_size").value
        self._dictionary_id_str = self.get_parameter("dictionary_id").value
        self._publish_tf = self.get_parameter("publish_tf").value
        self._base_frame = self.get_parameter("base_frame").value

        # Camera calibration data
        self._camera_info_msg = None
        self._camera_intrinsic_mat = None
        self._camera_distortion = None

        # Initialize CV Bridge
        self._bridge = CvBridge()

        # Subscribe to the camera image topic with a reliable QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._camera_raw_subscription = self.create_subscription(
            Image, self._camera_raw_topic, self.image_callback, qos_profile
        )
        # Camera info subscription
        self._camera_info_subscriber = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_callback, qos_profile
        )
        self.publisher_ = self.create_publisher(Image, "aruco_detection_image", 10)

        # Initialize ArUco detector
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self._dictionary_id_str)
        )
        self._detector_parameters = cv2.aruco.DetectorParameters_create()

        # TF Broadcaster
        if self._publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize TF buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # Create a timer for checking transforms
        self._transform_check_timer = self.create_timer(0.5, self.check_transforms)
        # Keep track of detected marker IDs
        self._detected_marker_ids = set()

        self.get_logger().info(
            f"ArUco detector node started. Subscribing to {self._camera_raw_topic}"
        )
        self.get_logger().info(
            f"Using dictionary: {self._dictionary_id_str}, marker size: {self._marker_size} meters"
        )
        self.get_logger().info(f"Base frame: {self._base_frame}")
        self.get_logger().info("TF listener initialized and ready")

    def check_transforms(self):
        """
        Periodically check for transforms from detected ArUco markers
        and perform additional processing when they're available.
        """

        # Skip if no markers have been detected yet
        if not self._detected_marker_ids:
            return

        for marker_id in list(self._detected_marker_ids):
            marker_frame = f"aruco_marker_{marker_id}"

            self.get_logger().info(
                "check_transforms called, marker frame: " + marker_frame
            )
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    marker_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1),
                )

                # Process the transform data
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z

                # Calculate distance from origin of base frame to marker
                distance = np.sqrt(tx * tx + ty * ty + tz * tz)

                # Log the information
                self.get_logger().info(
                    f"Marker {marker_id} is at position [{tx:.3f}, {ty:.3f}, {tz:.3f}] "
                    f"relative to {self._base_frame} (distance: {distance:.3f}m)"
                )

                # Example: You could publish this data to another topic
                # or use it to make decisions in your application

            except Exception as e:
                # If transform isn't available yet or has expired, don't report an error
                if "lookup would require extrapolation" not in str(e):
                    self.get_logger().debug(
                        f"Cannot look up transform for marker {marker_id}: {e}"
                    )

    def _camera_info_callback(self, info_msg):
        """Process camera calibration information."""
        self._camera_info_msg = info_msg
        # Reshape the 9-element k array into a 3x3 matrix
        self._camera_intrinsic_mat = np.array(self._camera_info_msg.k).reshape(3, 3)
        # Get the distortion coefficients directly
        self._camera_distortion = np.array(self._camera_info_msg.d)

        # Log the received calibration to verify
        self.get_logger().info(
            f"Camera intrinsic matrix: \n{self._camera_intrinsic_mat}"
        )
        self.get_logger().info(f"Distortion coefficients: {self._camera_distortion}")
        self.get_logger().info("Camera calibration received")

        # Unsubscribe from the camera info topic since we don't need it anymore
        self.destroy_subscription(self._camera_info_subscriber)
        self.get_logger().info("Unsubscribed from camera info topic")

    def image_callback(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Check if we have camera calibration data
        if self._camera_intrinsic_mat is None or self._camera_distortion is None:
            self.get_logger().warn("No camera calibration data available yet")
            return
        else:
            # Use the calibration data from camera_info
            camera_matrix = self._camera_intrinsic_mat
            dist_coeffs = self._camera_distortion

        # Detect ArUco markers
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
            cv_image, self._dictionary, parameters=self._detector_parameters
        )

        if ids is not None:
            self.get_logger().debug(f"Detected ArUco markers with IDs: {ids}")

            # Estimate pose of each detected marker
            # Use the camera matrix and distortion coefficients from camera info
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self._marker_size, camera_matrix, dist_coeffs
            )

            # Publish TF transforms for each detected marker
            if self._publish_tf:
                for i in range(len(ids)):
                    if rvecs is not None and tvecs is not None:
                        rvec = rvecs[i][0]
                        tvec = tvecs[i][0]
                        marker_id = ids[i][0]

                        # Add this marker ID to our tracking set
                        self._detected_marker_ids.add(marker_id)

                        # Log transformation details for debugging
                        self.get_logger().debug(
                            f"Marker {marker_id} translation: [{tvec[0]:.4f}, {tvec[1]:.4f}, {tvec[2]:.4f}]"
                        )
                        self.get_logger().debug(
                            f"Marker {marker_id} rotation vector: [{rvec[0]:.4f}, {rvec[1]:.4f}, {rvec[2]:.4f}]"
                        )

                        # Create TF transform message
                        transform_stamped = TransformStamped()
                        transform_stamped.header.stamp = self.get_clock().now().to_msg()
                        # Use the frame_id from the camera message
                        transform_stamped.header.frame_id = msg.header.frame_id
                        transform_stamped.child_frame_id = f"aruco_marker_{marker_id}"

                        # Translation
                        transform_stamped.transform.translation.x = float(tvec[0])
                        transform_stamped.transform.translation.y = float(tvec[1])
                        transform_stamped.transform.translation.z = float(tvec[2])

                        # Rotation (convert rotation vector to quaternion)
                        rotation_matrix = np.eye(3)
                        cv2.Rodrigues(rvec, rotation_matrix)
                        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                        transform_stamped.transform.rotation.x = float(quaternion[0])
                        transform_stamped.transform.rotation.y = float(quaternion[1])
                        transform_stamped.transform.rotation.z = float(quaternion[2])
                        transform_stamped.transform.rotation.w = float(quaternion[3])

                        # Print detailed transform information for debugging
                        self.get_logger().debug(
                            f"Publishing transform for marker {marker_id}:\n"
                            f"  Translation: [{tvec[0]:.4f}, {tvec[1]:.4f}, {tvec[2]:.4f}]\n"
                            f"  Rotation (quaternion): [{quaternion[0]:.4f}, {quaternion[1]:.4f}, {quaternion[2]:.4f}, {quaternion[3]:.4f}]\n"
                            f"  From frame: {transform_stamped.header.frame_id}\n"
                            f"  To frame: {transform_stamped.child_frame_id}"
                        )

                        # Broadcast the transform
                        self.tf_broadcaster.sendTransform(transform_stamped)

            # Draw detected markers on the image
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            if rvecs is not None and tvecs is not None:
                for i in range(len(ids)):
                    cv2.drawFrameAxes(
                        cv_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05
                    )  # Draw axis for each marker

        # Publish the image with detected markers
        try:
            detection_image_msg = self._bridge.cv2_to_imgmsg(cv_image, "bgr8")
            detection_image_msg.header.stamp = self.get_clock().now().to_msg()
            detection_image_msg.header.frame_id = (
                msg.header.frame_id
            )  # Ensure frame ID is set
            self.publisher_.publish(detection_image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing detection image: {e}")

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert a rotation matrix to a quaternion using a more robust method."""
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        elif (
            rotation_matrix[0, 0] > rotation_matrix[1, 1]
            and rotation_matrix[0, 0] > rotation_matrix[2, 2]
        ):
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
                - rotation_matrix[2, 2]
            )
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[1, 1]
                - rotation_matrix[0, 0]
                - rotation_matrix[2, 2]
            )
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[2, 2]
                - rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
            )
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            qz = 0.25 * s

        return [qx, qy, qz, qw]


def main(args=None):
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        # Create and initialize the node
        node = ArucoDetectorNode()
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
