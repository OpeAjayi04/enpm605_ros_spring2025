#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters_demo_file = PathJoinSubstitution(
        [FindPackageShare("parameters_demo"), "config", "parameters_demo.yaml"]
    )

    # Create launch configuration variables
    processing_mode = LaunchConfiguration("processing_mode")
    processing_rate = LaunchConfiguration("processing_rate")

    # Declare launch arguments with default values
    processing_mode_arg = DeclareLaunchArgument(
        "processing_mode",
        default_value="all",
        description="Processing mode (all, radar_only, camera_only, lidar_only)",
    )

    processing_rate_arg = DeclareLaunchArgument(
        "processing_rate", default_value="10", description="Processing rate in Hz"
    )

    # Define sensor nodes
    camera_node = Node(
        package="parameters_demo",
        executable="camera_demo",
        parameters=[parameters_demo_file],
        output="screen",
        emulate_tty=True,
    )

    lidar_node = Node(
        package="parameters_demo",
        executable="lidar_demo",
        parameters=[parameters_demo_file],
        output="screen",
        emulate_tty=True,
    )

    radar_node = Node(
        package="parameters_demo",
        executable="radar_demo",
        parameters=[parameters_demo_file],
        output="screen",
        emulate_tty=True,
    )

    # Define processing node with launch arguments as parameters
    processing_node = Node(
        package="parameters_demo",
        executable="processing_demo",
        parameters=[
            parameters_demo_file,
            {  # Override with command-line LaunchConfiguration values
                "processing_mode": processing_mode,
                "processing_rate": processing_rate,
            },
        ],
        output="screen",
        emulate_tty=True,
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
