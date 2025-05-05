# enpm605_ros_spring2025
ROS packages for ENPM605 (Spring 2025)

# Lecture 14

## ros2-system-monitor

## rosgpt

## ROSAnthropic Nav2

A ROS2 package that enables natural language robot navigation using Anthropic's Claude AI and the Navigation2 stack.

### Overview

ROSAnthropic Nav2 creates a bridge between natural language commands and robotic navigation. Users can control a ROS2-enabled robot using simple spoken or typed commands such as "Go to top_right_corner" or "Follow a path through the top left corner, top right corner, and bottom h."

The system leverages Anthropic's Claude AI to parse natural language commands and convert them into structured navigation instructions that the Navigation2 stack can execute.

### Components

The system consists of three main Python scripts:

1. **ros_anthropic_nav2_server.py**: Provides a web server interface and connects to Claude AI
2. **ros_anthropic_nav2_control.py**: Controls robot navigation using the Nav2 stack
3. **ros_anthropic_nav2_client.py**: Command-line interface for sending navigation commands

```mermaid
graph LR
    A[User Command] --> B[Client]
    B --> C[Server]
    C --> D[Claude AI]
    D --> E[ROS2 Topic]
    E --> F[Nav2 Controller]
    F --> G[Robot Movement]
```

### Features

- Natural language command interpretation
- Multiple navigation modes:
  - Direct navigation to a single destination
  - Sequential waypoint navigation (with stops at each point)
  - Continuous path following (without stopping)
- Web interface for command input
- Text-to-speech feedback (optional)
- Robust error handling and graceful degradation

### Available Locations

The system is configured with the following locations:
- top_right_corner
- top_left_corner
- bottom_left_corner
- bottom_right_corner
- top_h
- bottom_h

### Installation

Install required dependencies:

#### Python Dependencies

```bash
pip install anthropic flask flask-restful flask-cors rclpy requests pyttsx3 transforms3d
```

#### ROS2 Dependencies

- Pull packages from the repository.
```bash
# Build the packages
colcon build --symlink-install --packages-select mapping_navigation_demo rosgpt

# Start the simulation environment
ros2 launch mapping_navigation_demo navigation_with_map_corrected.launch.py

# Start the controller
ros2 run rosgpt anthropic_nav2_control

# Start the server
ros2 run rosgpt anthropic_nav2_server

# Start the client
ros2 run rosgpt anthropic_nav2_client

```

```bash
# ROS2 dependencies
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup libespeak1

# Python dependencies
pip install anthropic flask flask-restful flask-cors rclpy requests pyttsx3 transforms3d
```