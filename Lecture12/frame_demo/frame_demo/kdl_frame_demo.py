import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import PyKDL
import math
import tf_transformations
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ArUcoDetectionDemo(Node):
    """
    Class to demonstrate frame transformations using ArUco markers in ROS2.

    This class processes camera feeds, detects ArUco markers, and computes
    their poses in the odom coordinate frame using the KDL library.
    
    Attributes:
        _marker_poses_in_camera (dict): Poses of markers in camera frame
        _camera_pose_in_odom (Pose): Pose of camera in odom frame
    """

    def __init__(self, node_name="aruco_detection_demo"):
        """
        Initialize the ArUcoDetectionDemo node.

        Args:
            node_name (str): Name of the ROS node
        """
        super().__init__(node_name)

        # Initialize dictionaries to store marker poses
        self._marker_poses_in_camera = {}
        
        # Camera pose in odom frame
        self._camera_pose_in_odom = None
        
        # Camera info
        self._camera_intrinsic_mat = None
        self._camera_distortion = None
        self._camera_info_received = False
        
        # Parameters
        self.declare_parameter("marker_size", 0.194)  # ArUco marker size in meters
        self.declare_parameter("dictionary_id", "DICT_5X5_250")  # ArUco dictionary
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("odom_frame", "odom")
        
        # Get parameters
        self._marker_size = self.get_parameter("marker_size").value
        self._dictionary_id_str = self.get_parameter("dictionary_id").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        
        # Initialize CV Bridge
        self._bridge = CvBridge()
        
        # Initialize ArUco detector
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self._dictionary_id_str)
        )
        self._detector_parameters = cv2.aruco.DetectorParameters_create()
        
        # QoS profile for camera topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Create subscribers
        self._image_sub = self.create_subscription(
            Image, 
            "/camera/color/image_raw", 
            self._image_callback, 
            qos_profile
        )
        
        self._camera_info_sub = self.create_subscription(
            CameraInfo, 
            "/camera/color/camera_info", 
            self._camera_info_callback, 
            qos_profile
        )
        
        # TF system setup
        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        self.get_logger().info("ArUco detection demo started")
        self.get_logger().info(f"Looking for ArUco markers using {self._dictionary_id_str}")
        self.get_logger().info(f"Marker size: {self._marker_size} meters")

    def _camera_info_callback(self, msg):
        """
        Process camera calibration information.
        
        Args:
            msg (CameraInfo): Camera calibration data
        """
        if not self._camera_info_received:
            # Extract camera matrix from camera info
            self._camera_intrinsic_mat = np.array(msg.k).reshape(3, 3)
            
            # Extract distortion coefficients
            self._camera_distortion = np.array(msg.d)
            
            self._camera_info_received = True
            self.get_logger().info(f"Camera intrinsic matrix: \n{self._camera_intrinsic_mat}")
            self.get_logger().info(f"Distortion coefficients: {self._camera_distortion}")
            self.get_logger().info("Camera calibration received")

    def _image_callback(self, msg):
        """
        Process camera images and detect ArUco markers.
        
        Args:
            msg (Image): Camera image
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Check if we have camera calibration data
            if not self._camera_info_received:
                self.get_logger().warn("No camera calibration data available yet")
                return
            
            # Get current camera pose in odom frame
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._odom_frame,
                    self._camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Store camera pose in odom frame
                self._camera_pose_in_odom = Pose()
                self._camera_pose_in_odom.position.x = transform.transform.translation.x
                self._camera_pose_in_odom.position.y = transform.transform.translation.y
                self._camera_pose_in_odom.position.z = transform.transform.translation.z
                self._camera_pose_in_odom.orientation = transform.transform.rotation
            except Exception as e:
                self.get_logger().debug(f"Could not get camera pose: {e}")
                return
            
            # Detect ArUco markers
            corners, ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self._dictionary, parameters=self._detector_parameters
            )
            
            # Clear previous marker poses
            self._marker_poses_in_camera.clear()
            
            if ids is not None and self._camera_pose_in_odom is not None:
                self.get_logger().debug(f"Detected ArUco markers with IDs: {ids}")
                
                # Estimate pose of each marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self._marker_size, self._camera_intrinsic_mat, self._camera_distortion
                )
                
                # Process each detected marker
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]
                    
                    # Create pose for marker in camera frame
                    pose_in_camera = Pose()
                    pose_in_camera.position.x = float(tvec[0])
                    pose_in_camera.position.y = float(tvec[1])
                    pose_in_camera.position.z = float(tvec[2])
                    
                    # Convert rotation vector to quaternion
                    rot_mat = np.eye(3)
                    cv2.Rodrigues(rvec, rot_mat)
                    quat = self._rotation_matrix_to_quaternion(rot_mat)
                    
                    pose_in_camera.orientation.x = float(quat[0])
                    pose_in_camera.orientation.y = float(quat[1])
                    pose_in_camera.orientation.z = float(quat[2])
                    pose_in_camera.orientation.w = float(quat[3])
                    
                    # Store marker pose in camera frame
                    self._marker_poses_in_camera[marker_id] = pose_in_camera
                    
                    # Compute marker pose in odom frame
                    pose_in_odom = self._compute_marker_pose_in_odom(
                        pose_in_camera, self._camera_pose_in_odom
                    )
                    
                    # Log marker pose information
                    self._log_pose_information(marker_id, pose_in_camera, pose_in_odom)
        
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def _compute_marker_pose_in_odom(self, marker_pose_in_camera, camera_pose_in_odom):
        """
        Transform a marker's pose from camera frame to odom frame.
        
        This function performs a coordinate frame transformation using KDL frames.
        
        Args:
            marker_pose_in_camera (Pose): Pose of marker in camera frame
            camera_pose_in_odom (Pose): Pose of camera in odom frame
            
        Returns:
            Pose: Pose of marker in odom frame
        """
        # Create KDL Frame for camera-to-odom transformation
        camera_orientation = camera_pose_in_odom.orientation
        camera_position = camera_pose_in_odom.position
        kdl_camera_odom = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ),
            PyKDL.Vector(camera_position.x, camera_position.y, camera_position.z),
        )
        
        # Create KDL Frame for marker-to-camera transformation
        marker_orientation = marker_pose_in_camera.orientation
        marker_position = marker_pose_in_camera.position
        kdl_marker_camera = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                marker_orientation.x,
                marker_orientation.y,
                marker_orientation.z,
                marker_orientation.w,
            ),
            PyKDL.Vector(marker_position.x, marker_position.y, marker_position.z),
        )
        
        # Compute marker-to-odom transformation
        kdl_marker_odom = kdl_camera_odom * kdl_marker_camera
        
        # Convert back to Pose message
        pose = Pose()
        pose.position.x = kdl_marker_odom.p.x()
        pose.position.y = kdl_marker_odom.p.y()
        pose.position.z = kdl_marker_odom.p.z()
        
        # Extract quaternion
        q = kdl_marker_odom.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        return pose

    def _log_pose_information(self, marker_id, pose_in_camera, pose_in_odom):
        """
        Log marker pose information in both camera and odom frames.
        
        Args:
            marker_id (int): ID of the ArUco marker
            pose_in_camera (Pose): Pose of the marker in camera frame
            pose_in_odom (Pose): Pose of the marker in odom frame
        """
        # Only log occasionally to avoid flooding
        now = self.get_clock().now()
        if (not hasattr(self, f"_last_log_time_{marker_id}") or 
            (now - getattr(self, f"_last_log_time_{marker_id}")).nanoseconds / 1e9 > 1.0):
            
            # Pose in camera frame
            pos_camera = pose_in_camera.position
            quat_camera = [
                pose_in_camera.orientation.x,
                pose_in_camera.orientation.y,
                pose_in_camera.orientation.z,
                pose_in_camera.orientation.w
            ]
            euler_camera = tf_transformations.euler_from_quaternion(quat_camera)
            rpy_camera = [math.degrees(angle) for angle in euler_camera]
            
            # Pose in odom frame
            pos_odom = pose_in_odom.position
            quat_odom = [
                pose_in_odom.orientation.x,
                pose_in_odom.orientation.y,
                pose_in_odom.orientation.z,
                pose_in_odom.orientation.w
            ]
            euler_odom = tf_transformations.euler_from_quaternion(quat_odom)
            rpy_odom = [math.degrees(angle) for angle in euler_odom]
            
            output = "\n" + "=" * 50 + "\n"
            output += f"ArUco marker {marker_id}:\n"
            output += f"  In camera frame ({self._camera_frame}):\n"
            output += f"    Position: [{pos_camera.x:.3f}, {pos_camera.y:.3f}, {pos_camera.z:.3f}]\n"
            output += f"    Orientation (RPY): [{rpy_camera[0]:.1f}°, {rpy_camera[1]:.1f}°, {rpy_camera[2]:.1f}°]\n\n"
            output += f"  In odom frame ({self._odom_frame}):\n"
            output += f"    Position: [{pos_odom.x:.3f}, {pos_odom.y:.3f}, {pos_odom.z:.3f}]\n"
            output += f"    Orientation (RPY): [{rpy_odom[0]:.1f}°, {rpy_odom[1]:.1f}°, {rpy_odom[2]:.1f}°]\n"
            output += "=" * 50 + "\n"
            self.get_logger().info(output)
            
            setattr(self, f"_last_log_time_{marker_id}", now)

    def _rotation_matrix_to_quaternion(self, rotation_matrix):
        """
        Convert a rotation matrix to a quaternion.
        
        Args:
            rotation_matrix (numpy.ndarray): 3x3 rotation matrix
            
        Returns:
            list: Quaternion [x, y, z, w]
        """
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            qz = 0.25 * s
            
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    
    aruco_detector = ArUcoDetectionDemo()
    
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()