#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Generate a launch description for running multiple camera nodes with different namespaces.
    
    This launch file demonstrates namespace-based remapping by running two instances of the
    same camera_demo node in different namespaces.
    """
    # Create launch configuration variables
    right_camera_rate = LaunchConfiguration('right_cam_rate')
    left_camera_rate = LaunchConfiguration('left_cam_rate')
    rear_camera_rate = LaunchConfiguration('rear_cam_rate')
    right_camera_frame = LaunchConfiguration('right_cam_frame')
    left_camera_frame = LaunchConfiguration('left_cam_frame')
    rear_camera_frame = LaunchConfiguration('rear_cam_frame')

    # Declare launch arguments
    right_camera_rate_arg = DeclareLaunchArgument(
        'right_cam_rate',
        default_value='40',
        description='Publishing rate for right camera in Hz'
    )
    
    left_camera_rate_arg = DeclareLaunchArgument(
        'left_cam_rate',
        default_value='40',
        description='Publishing rate for left camera in Hz'
    )
    
    rear_camera_rate_arg = DeclareLaunchArgument(
        'rear_cam_rate',
        default_value='20',
        description='Publishing rate for rear camera in Hz'
    )
    
    right_camera_frame_arg = DeclareLaunchArgument(
        'right_cam_frame',
        default_value='camera_id',
        description='Frame ID for the right camera'
    )
    
    left_camera_frame_arg = DeclareLaunchArgument(
        'left_cam_frame',
        default_value='camera_id',
        description='Frame ID for the left camera'
    )
    
    rear_camera_frame_arg = DeclareLaunchArgument(
        'rear_cam_frame',
        default_value='camera_id',
        description='Frame ID for the rear camera'
    )
    
    right_camera_node = Node(
        package="remapping_demo",
        executable="camera_demo",
        namespace='vehicle',
        name="right_camera",
        remappings=[
            ("camera/image_color", "right/image_color"),
        ],
        parameters=[{
        'camera_name': 'right_camera',
        'camera_rate': right_camera_rate,
        'camera_frame': right_camera_frame
        }],
        output="screen",
        emulate_tty=True,
    )

    left_camera_node = Node(
        package="remapping_demo",
        executable="camera_demo",
        namespace='vehicle',
        name="left_camera",
        remappings=[
            ("camera/image_color", "left/image_color"),
        ],
        parameters=[{
        'camera_name': 'left_camera',
        'camera_rate': left_camera_rate,
        'camera_frame': left_camera_frame
        }],
        output="screen",
        emulate_tty=True,
    )

    rear_camera_node = Node(
        package="remapping_demo",
        executable="camera_demo",
        namespace='vehicle',
        name="rear_camera",
        remappings=[
            ("camera/image_color", "rear/image_color"),
        ],
        parameters=[{
        'camera_name': 'rear_camera',
        'camera_rate': rear_camera_rate,
        'camera_frame': rear_camera_frame
        }],
        output="screen",
        emulate_tty=True,
    )
    
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(right_camera_rate_arg)
    ld.add_action(left_camera_rate_arg)
    ld.add_action(rear_camera_rate_arg)
    ld.add_action(right_camera_frame_arg)
    ld.add_action(left_camera_frame_arg)
    ld.add_action(rear_camera_frame_arg)
    
    # Add nodes
    ld.add_action(right_camera_node)
    ld.add_action(left_camera_node)
    ld.add_action(rear_camera_node)
    
    return ld