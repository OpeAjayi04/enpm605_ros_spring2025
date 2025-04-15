from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory("rosbot_navigation_demo")

    # Define paths
    goals_path = os.path.join(package_dir, "config", "goals.yaml")

    # Launch arguments
    map_frame_arg = DeclareLaunchArgument(
        "map_frame", default_value="odom", description="Frame ID for the map"
    )

    rotation_speed_arg = DeclareLaunchArgument(
        "rotation_speed",
        default_value="0.3",
        description="Robot rotation speed when searching for color",
    )

    target_color_arg = DeclareLaunchArgument(
        "target_color", default_value="red", description="Target color for navigation"
    )

    # Nodes
    color_beacon_node = Node(
        package="rosbot_navigation_demo",
        executable="color_beacon_demo",
        output="screen",
        parameters=[
            {
                "red_lower_hsv": [0, 30, 30],
                "red_upper_hsv": [15, 255, 255],
                "red_lower_hsv2": [160, 30, 30],
                "red_upper_hsv2": [179, 255, 255],
                "min_detection_area": 30,
                "rotation_speed": LaunchConfiguration("rotation_speed"),
                "rotation_timer_period": 0.1,
            }
        ],
    )

    goal_provider_node = Node(
        package="rosbot_navigation_demo",
        executable="goal_provider_service_demo",
        output="screen",
        parameters=[
            {"goals_file": goals_path, "map_frame": LaunchConfiguration("map_frame")}
        ],
    )

    navigation_controller_node = Node(
        package="rosbot_navigation_demo",
        executable="navigation_controller_demo",
        output="screen",
        parameters=[
            {
                "target_color": LaunchConfiguration("target_color"),
                "cooldown_seconds": 5.0,
                "distance_tolerance": 0.1,
                "angle_tolerance": 0.05,
            }
        ],
    )

    move_to_goal_action_server_node = Node(
        package="rosbot_navigation_demo",
        executable="move_to_goal_action_demo",
        output="screen",
        parameters=[
            {
                "kp_linear": 0.5,
                "kp_angular": 1.0,
                "default_linear_tolerance": 0.1,
                "default_angular_tolerance": 0.05,
                "control_frequency": 20.0,
            }
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(map_frame_arg)
    ld.add_action(rotation_speed_arg)
    ld.add_action(target_color_arg)

    # Add nodes
    ld.add_action(color_beacon_node)
    ld.add_action(goal_provider_node)
    ld.add_action(navigation_controller_node)
    ld.add_action(move_to_goal_action_server_node)

    return ld
