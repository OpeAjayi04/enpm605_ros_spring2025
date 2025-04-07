#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate a launch description for running multiple camera nodes with different namespaces.
    
    This launch file demonstrates namespace-based remapping by running three instances of the
    same camera_demo node in different namespaces.
    """
    
    right_camera_node = Node(
        package='remapping_demo',
        executable='camera_demo',
        namespace='right_camera',
        output='screen',
        emulate_tty=True
    )
    
    left_camera_node = Node(
        package='remapping_demo',
        executable='camera_demo',
        namespace='left_camera',
        output='screen',
        emulate_tty=True
    )
    
    rear_camera_node = Node(
        package='remapping_demo',
        executable='camera_demo',
        namespace='rear_camera',
        output='screen',
        emulate_tty=True
    )
    
    # Create the launch description
    ld = LaunchDescription()

    # Add nodes
    ld.add_action(right_camera_node)
    ld.add_action(left_camera_node)
    ld.add_action(rear_camera_node)
    
    return ld