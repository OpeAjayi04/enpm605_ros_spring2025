#!/usr/bin/env python3
# This file is part of rosanthropic package.
#
# Copyright (c) 2023-2025 Anis Koubaa.
# Modified for Anthropic Claude AI integration with Nav2.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import logging
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations
from rclpy.parameter import Parameter

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger('rosanthropic_nav2')

class ROSAnthropicNav2Control(Node):
    """A ROS2 node that controls a robot using Nav2 based on voice commands."""

    def __init__(self):
        """Initialize the ROSAnthropicNav2 node."""
        super().__init__('rosanthropic_nav2')
        
        # Set use_sim_time parameter if using simulation
        sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])
        
        # Create subscription for voice commands
        self.create_subscription(
            String, 
            '/voice_cmd', 
            self.voice_cmd_callback, 
            10
        )
        
        # Flag to track if navigation is in progress
        self.is_navigating = False
        
        # Map of named locations to coordinates (x, y, orientation)
        self.locations = {
            "top_right_corner": (9.5, -9.5, 0.0),
            "living room": (5.0, 2.0, 0.0),
            "bedroom": (8.0, 3.0, 0.0),
            "bathroom": (3.0, 7.0, 0.0),
            "dining room": (6.0, 6.0, 0.0),
            "office": (9.0, 5.0, 0.0),
            "entry": (0.5, 0.5, 0.0),
            "charging station": (0.0, 0.0, 0.0),
        }
        
        # Initialize the navigator
        self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)
        
        logger.info('ROSAnthropic Nav2 Controller Started. Initializing navigation...')

    def _initialize_navigation_callback(self):
        """Initialize the Nav2 navigation system."""
        # Cancel the timer so this only runs once
        self._init_timer.cancel()
        
        try:
            # Create a BasicNavigator instance
            self.navigator = BasicNavigator()
            
            # Set initial pose
            self.localize()
            
            # Create a timeout mechanism ourselves since the built-in one isn't available
            start_time = time.time()
            timeout = 10.0  # 10 second timeout
            
            try:
                # Try to wait for Nav2 to become active with our own timeout
                self.get_logger().info("Waiting for Nav2 to become active...")
                while time.time() - start_time < timeout:
                    if self.navigator._waitForNodeToActivate('amcl'):
                        break
                    time.sleep(0.5)
                
                # Check if we timed out
                if time.time() - start_time >= timeout:
                    self.get_logger().warn("Nav2 did not become active in the expected timeframe. Will retry initialization later.")
                    self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)
                    return
                
                # Wait for Nav2 to be ready (with standard method that doesn't take timeout)
                self.navigator.waitUntilNav2Active()
                self.get_logger().info('Navigation initialized successfully. Waiting for commands...')
            except Exception as e:
                self.get_logger().warn(f"Error waiting for Nav2: {str(e)}. Will retry initialization.")
                self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)
                return
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize navigation: {str(e)}")
            # Try again after a delay
            self._init_timer = self.create_timer(5.0, self._initialize_navigation_callback)

    def localize(self):
        """Set the initial pose of the robot."""
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        
        # Publish initial pose multiple times to ensure it's received
        for _ in range(3):
            self.navigator.setInitialPose(initial_pose)
            time.sleep(0.5)
        
        self.get_logger().info("Initial pose set")

    def voice_cmd_callback(self, msg):
        """Parse voice commands in JSON format and execute corresponding actions."""
        try:
            # Parse the JSON data
            cmd_data = json.loads(msg.data)
            
            # Extract the JSON command - handling both formats
            if isinstance(cmd_data, dict) and 'json' in cmd_data:
                # Handle JSON string within text wrapper format
                cmd = json.loads(cmd_data['json'])
            else:
                # Direct JSON format
                cmd = cmd_data
                
            logger.info(f'JSON command received: {json.dumps(cmd, indent=2)}')
            
            # Check if already navigating
            if self.is_navigating:
                logger.warning("Already navigating. Please wait for completion.")
                return
            
            # Process based on action type
            if cmd['action'] == 'go_to_goal':
                if 'params' in cmd and 'location' in cmd['params'] and 'value' in cmd['params']['location']:
                    location = cmd['params']['location']['value']
                    self.go_to_goal(location)
                else:
                    logger.error("Malformed 'go_to_goal' command: missing location parameter")
            
            elif cmd['action'] == 'sequence':
                # Handle sequence of waypoints
                if 'params' in cmd and isinstance(cmd['params'], list):
                    waypoints = []
                    for action in cmd['params']:
                        if action['action'] == 'go_to_goal' and 'location' in action['params'] and 'value' in action['params']['location']:
                            location = action['params']['location']['value']
                            if location.lower() in self.locations:
                                waypoints.append(location.lower())
                    
                    if waypoints:
                        # Check if continuous path parameter is set
                        continuous_path = False
                        for action in cmd['params']:
                            if 'continuous_path' in action and isinstance(action['continuous_path'], bool):
                                continuous_path = action['continuous_path']
                                break
                        
                        if continuous_path:
                            self.follow_path(waypoints)
                        else:
                            self.follow_waypoints(waypoints)
                    else:
                        logger.error("No valid waypoints found in sequence")
                else:
                    logger.error("Malformed 'sequence' command: 'params' must be a list of actions")
            
            elif cmd['action'] == 'follow_path':
                # Direct continuous path following command
                if 'params' in cmd and 'locations' in cmd['params'] and isinstance(cmd['params']['locations'], list):
                    locations = cmd['params']['locations']
                    waypoints = []
                    for location in locations:
                        if isinstance(location, dict) and 'value' in location:
                            loc_name = location['value'].lower()
                            if loc_name in self.locations:
                                waypoints.append(loc_name)
                    
                    if waypoints:
                        self.follow_path(waypoints)
                    else:
                        logger.error("No valid locations found in follow_path command")
                else:
                    logger.error("Malformed 'follow_path' command: missing or invalid locations parameter")
            else:
                logger.warning(f"Unknown or unsupported action: {cmd['action']}")
                
        except json.JSONDecodeError:
            logger.error(f"Invalid or empty JSON string received: {msg.data}")
        except KeyError as e:
            logger.error(f"Missing key in JSON: {e}")
        except Exception as e:
            logger.error(f"Unexpected error processing command: {str(e)}")
    
    def create_pose_stamped(self, x, y, yaw):
        """Create a PoseStamped message with the given coordinates and orientation."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        
        return goal_pose
    
    def go_to_goal(self, location):
        """Navigate to a specified location using Nav2."""
        try:
            self.is_navigating = True
            
            if location.lower() not in self.locations:
                logger.warning(f"Unknown location: {location}. Going to charging station instead.")
                x_goal, y_goal, yaw_goal = self.locations["charging station"]
            else:
                x_goal, y_goal, yaw_goal = self.locations[location.lower()]
            
            logger.info(f"Navigating to location '{location}' at coordinates ({x_goal}, {y_goal})")
            
            # Wait for Nav2 to be ready
            self.navigator.waitUntilNav2Active()
            
            # Create goal pose
            goal_pose = self.create_pose_stamped(x_goal, y_goal, yaw_goal)
            
            # Send the goal
            self.navigator.goToPose(goal_pose)
            
            # Wait for navigation to complete
            while not self.navigator.isTaskComplete():
                # Get feedback (optional)
                feedback = self.navigator.getFeedback()
                if feedback and hasattr(feedback, 'distance_remaining'):
                    logger.info(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
                time.sleep(1.0)
            
            # Get the result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                logger.info(f"Successfully reached location '{location}'")
            elif result == TaskResult.CANCELED:
                logger.warning(f"Navigation to '{location}' was canceled")
            elif result == TaskResult.FAILED:
                logger.error(f"Failed to reach location '{location}'")
            
        except Exception as e:
            logger.error(f"Error during go_to_goal operation: {str(e)}")
        finally:
            self.is_navigating = False
    
    def follow_waypoints(self, waypoint_locations):
        """Follow a list of waypoints using Nav2 - stops at each waypoint."""
        try:
            self.is_navigating = True
            logger.info(f"Following {len(waypoint_locations)} waypoints with stops at each point")
            
            # Wait for Nav2 to be ready
            self.navigator.waitUntilNav2Active()
            
            # Convert location names to PoseStamped objects
            pose_waypoints = []
            for loc in waypoint_locations:
                if loc in self.locations:
                    x, y, yaw = self.locations[loc]
                    pose = self.create_pose_stamped(x, y, yaw)
                    pose_waypoints.append(pose)
                else:
                    logger.warning(f"Unknown location: {loc}. Skipping this waypoint.")
            
            if not pose_waypoints:
                logger.error("No valid waypoints found. Aborting follow_waypoints.")
                self.is_navigating = False
                return
            
            # Send the waypoints
            self.navigator.followWaypoints(pose_waypoints)
            
            # Wait for navigation to complete
            while not self.navigator.isTaskComplete():
                # Get feedback (optional)
                feedback = self.navigator.getFeedback()
                if feedback and hasattr(feedback, 'current_waypoint'):
                    current_wp = feedback.current_waypoint
                    logger.info(f"Following waypoint {current_wp+1}/{len(pose_waypoints)}")
                time.sleep(1.0)
            
            # Get the result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                logger.info("Successfully completed all waypoints")
            elif result == TaskResult.CANCELED:
                logger.warning("Waypoint navigation was canceled")
            elif result == TaskResult.FAILED:
                logger.error("Failed to complete all waypoints")
            
        except Exception as e:
            logger.error(f"Error during follow_waypoints operation: {str(e)}")
        finally:
            self.is_navigating = False
    
    def follow_path(self, waypoint_locations):
        """Follow a continuous path through waypoints without stopping at each waypoint."""
        try:
            self.is_navigating = True
            logger.info(f"Following continuous path through {len(waypoint_locations)} waypoints without stops")
            
            # Wait for Nav2 to be ready
            self.navigator.waitUntilNav2Active()
            
            # Convert location names to PoseStamped objects
            pose_waypoints = []
            for loc in waypoint_locations:
                if loc in self.locations:
                    x, y, yaw = self.locations[loc]
                    pose = self.create_pose_stamped(x, y, yaw)
                    pose_waypoints.append(pose)
                else:
                    logger.warning(f"Unknown location: {loc}. Skipping this waypoint.")
            
            if not pose_waypoints:
                logger.error("No valid waypoints found. Aborting follow_path.")
                self.is_navigating = False
                return
                
            # Create a path from waypoints
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.navigator.get_clock().now().to_msg()
            path.poses = pose_waypoints
            
            # Send the path for continuous following
            self.navigator.followPath(path)
            
            # Wait for navigation to complete
            while not self.navigator.isTaskComplete():
                # Get feedback (optional)
                feedback = self.navigator.getFeedback()
                if feedback and hasattr(feedback, 'distance_remaining'):
                    logger.info(f"Distance remaining on path: {feedback.distance_remaining:.2f} meters")
                time.sleep(1.0)
            
            # Get the result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                logger.info("Successfully completed continuous path")
            elif result == TaskResult.CANCELED:
                logger.warning("Path following was canceled")
            elif result == TaskResult.FAILED:
                logger.error("Failed to complete path")
            
        except Exception as e:
            logger.error(f"Error during follow_path operation: {str(e)}")
        finally:
            self.is_navigating = False


def main(args=None):
    """Main function to start the Nav2 controller node."""
    try:
        # Initialize ROS
        rclpy.init(args=args)
        
        # Create and spin the node
        node = ROSAnthropicNav2Control()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
    finally:
        # Clean shutdown
        rclpy.shutdown()
        logger.info("Nav2 controller shutdown complete")

if __name__ == '__main__':
    main()