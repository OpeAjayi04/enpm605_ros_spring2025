#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Create launch configuration variables
    processing_mode = LaunchConfiguration('processing_mode')
    processing_rate = LaunchConfiguration('processing_rate')
    
    # Declare launch arguments with default values
    processing_mode_arg = DeclareLaunchArgument(
        'processing_mode',
        default_value='all',
        description='Processing mode (all, radar_only, camera_only, lidar_only)'
    )
    
    processing_rate_arg = DeclareLaunchArgument(
        'processing_rate',
        default_value='10',
        description='Processing rate in Hz'
    )
    
    # Define sensor nodes
    camera_node = Node(
        package='parameters_demo',
        executable='camera_demo',
        # name='camera_demo_node',
        parameters=[
            {'camera_name': 'front_camera'},
            {'camera_rate': 30}
        ],
        output='screen',
        emulate_tty=True
    )
    
    lidar_node = Node(
        package='parameters_demo',
        executable='lidar_demo',
        # name='lidar_demo_node',
        parameters=[
            {'lidar_name': 'top_lidar'},
            {'lidar_rate': 20}
        ],
        output='screen',
        emulate_tty=True
    )
    
    radar_node = Node(
        package='parameters_demo',
        executable='radar_demo',
        # name='radar_scan_demo_node',
        parameters=[
            {'radar_name': 'front_radar'},
            {'radar_rate': 60}
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Define processing node with launch arguments as parameters
    processing_node = Node(
        package='parameters_demo',
        executable='processing_demo',
        # name='processing_demo_node',
        parameters=[
            {
                'processing_mode': processing_mode,
                'processing_rate': processing_rate
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Return the launch description
    # Create the LaunchDescription object
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(processing_mode_arg)
    ld.add_action(processing_rate_arg)
    
    # Add nodes
    ld.add_action(camera_node)
    ld.add_action(lidar_node)
    ld.add_action(radar_node)
    ld.add_action(processing_node)
    
    return ld