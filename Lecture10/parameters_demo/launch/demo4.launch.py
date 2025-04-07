#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    two_cameras_params_file = PathJoinSubstitution(
        [FindPackageShare("parameters_demo"), "config", "two_cameras_params.yaml"]
    )

    # Define sensor nodes
    camera1_node = Node(
        package="parameters_demo",
        executable="camera_demo",
        name="camera1_demo",
        remappings=[
            ("/camera/image_color", "/camera1/image_color"),
        ],
        parameters=[two_cameras_params_file],
        output="screen",
        emulate_tty=True,
    )

    # Define sensor nodes
    camera2_node = Node(
        package="parameters_demo",
        executable="camera_demo",
        name="camera2_demo",
        remappings=[
            ("/camera/image_color", "/camera2/image_color"),
        ],
        parameters=[two_cameras_params_file],
        output="screen",
        emulate_tty=True,
    )

    # Return the launch description
    # Create the LaunchDescription object
    ld = LaunchDescription()

    # Add nodes
    ld.add_action(camera1_node)
    ld.add_action(camera2_node)

    return ld
