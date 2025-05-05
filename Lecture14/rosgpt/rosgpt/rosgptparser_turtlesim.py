#!/usr/bin/env python3
# This file is part of rosanthropic package.
#
# Copyright (c) 2023 Anis Koubaa.
# Modified for Anthropic Claude AI integration.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
import time
import logging
import threading
from concurrent.futures import ThreadPoolExecutor
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger('rosanthropic_turtlesim')

class TurtlesimController(Node):
    """A ROS2 node that controls a turtlesim turtle based on voice commands."""

    def __init__(self):
        """Initialize the TurtlesimController node."""
        super().__init__('turtlesim_controller')
        
        # Create subscriptions and publishers
        self.create_subscription(
            String, 
            '/voice_cmd', 
            self.voice_cmd_callback, 
            10
        )
        self.velocity_publisher = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, 
            "/turtle1/pose", 
            self.pose_callback, 
            10
        )
        
        # Initialize turtle pose
        self.pose = Pose()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Create thread executor for non-blocking operations
        self.thread_executor = ThreadPoolExecutor(max_workers=2)
        
        # Setup executor for move operations
        self.move_executor = SingleThreadedExecutor()
        self.move_thread = threading.Thread(target=self.move_executor.spin)
        self.move_thread.daemon = True  # Make thread terminate when main thread exits
        self.move_thread.start()
        
        # Keep track of active operations
        self.is_moving = False
        self.is_rotating = False
        
        logger.info('ROSAnthropic Turtlesim Controller Started. Waiting for input commands...')

    def pose_callback(self, msg):
        """Update the current pose of the turtle."""
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.pose = msg
        # logger.debug(f"Current pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")

    def voice_cmd_callback(self, msg):
        """Parse voice commands in JSON format and execute corresponding actions."""
        try:
            # First, parse the JSON data
            cmd_data = json.loads(msg.data)
            
            # Extract the JSON command - handling both formats
            if isinstance(cmd_data, dict) and 'json' in cmd_data:
                # Handle JSON string within text wrapper format
                cmd = json.loads(cmd_data['json'])
            else:
                # Direct JSON format
                cmd = cmd_data
                
            logger.info(f'JSON command received: {json.dumps(cmd, indent=2)}')
            
            # Check if we're already executing a command
            if self.is_moving or self.is_rotating:
                logger.warning("Already executing a command. Please wait for completion.")
                return
            
            # Process based on action type
            if cmd['action'] == 'go_to_goal':
                if 'params' in cmd and 'location' in cmd['params'] and 'value' in cmd['params']['location']:
                    location = cmd['params']['location']['value']
                    self.thread_executor.submit(self.go_to_goal, location)
                else:
                    logger.error("Malformed 'go_to_goal' command: missing location parameter")
                    
            elif cmd['action'] == 'move':
                # Extract parameters with defaults
                linear_speed = cmd['params'].get('linear_speed', 0.2)
                distance = cmd['params'].get('distance', 1.0)
                is_forward = cmd['params'].get('is_forward', True)
                
                # Safety checks
                if linear_speed > 1.0:
                    logger.warning(f"Speed too high ({linear_speed}). Limiting to 1.0")
                    linear_speed = 1.0
                    
                # Execute move in separate thread
                self.thread_executor.submit(self.move, linear_speed, distance, is_forward)
                
            elif cmd['action'] == 'rotate':
                # Extract parameters with defaults
                angular_velocity = cmd['params'].get('angular_velocity', 10.0)  # in degrees per second
                angle = cmd['params'].get('angle', 90.0)  # in degrees
                is_clockwise = cmd['params'].get('is_clockwise', True)
                
                # Safety checks
                if angular_velocity > 30.0:
                    logger.warning(f"Angular velocity too high ({angular_velocity}). Limiting to 30.0 deg/s")
                    angular_velocity = 30.0
                
                # Execute rotate in separate thread
                self.thread_executor.submit(self.rotate, angular_velocity, angle, is_clockwise)
                
            elif cmd['action'] == 'sequence':
                # Handle sequence of actions
                if 'params' in cmd and isinstance(cmd['params'], list):
                    self.thread_executor.submit(self.execute_sequence, cmd['params'])
                else:
                    logger.error("Malformed 'sequence' command: 'params' must be a list of actions")
            
            else:
                logger.warning(f"Unknown action: {cmd['action']}")
                
        except json.JSONDecodeError:
            logger.error(f"Invalid or empty JSON string received: {msg.data}")
        except KeyError as e:
            logger.error(f"Missing key in JSON: {e}")
        except Exception as e:
            logger.error(f"Unexpected error processing command: {str(e)}")
    
    def get_distance(self, start, destination):
        """Calculate Euclidean distance between two poses."""
        return math.sqrt((destination.x - start.x)**2 + (destination.y - start.y)**2)
    
    def move(self, linear_speed, distance, is_forward):
        """Move the turtle in a straight line at the specified speed for the specified distance."""
        try:
            self.is_moving = True
            direction = 'forward' if is_forward else 'backward'
            logger.info(f"Moving {direction} at {linear_speed} m/s for {distance} meters")
            
            # Create and configure Twist message
            twist_msg = Twist()
            twist_msg.linear.x = float(abs(linear_speed) * (1 if is_forward else -1))
            
            # Save start position
            start_pose = copy.copy(self.pose)
            traveled_distance = 0.0
            
            # Move until desired distance is reached
            while traveled_distance < distance:
                # Check if position is updating properly
                traveled_distance = self.get_distance(start_pose, self.pose)
                logger.debug(f"Distance traveled: {traveled_distance:.2f}/{distance:.2f} meters")
                
                # Publish velocity command
                self.velocity_publisher.publish(twist_msg)
                
                # Let the executor process callbacks
                self.move_executor.spin_once(timeout_sec=0.1)
            
            # Stop the turtle
            twist_msg.linear.x = 0.0
            self.velocity_publisher.publish(twist_msg)
            
            # Final distance moved
            final_distance = self.get_distance(start_pose, self.pose)
            logger.info(f"Movement complete. Distance traveled: {final_distance:.2f} meters")
            
        except Exception as e:
            logger.error(f"Error during move operation: {str(e)}")
        finally:
            # Ensure we stop the turtle even if an error occurs
            stop_msg = Twist()
            self.velocity_publisher.publish(stop_msg)
            self.is_moving = False

    def rotate(self, angular_speed_degree, desired_angle_degree, clockwise):
        """Rotate the turtle at the specified angular velocity for the specified angle."""
        try:
            self.is_rotating = True
            direction = 'clockwise' if clockwise else 'counter-clockwise'
            logger.info(f"Rotating {direction} at {angular_speed_degree} deg/s for {desired_angle_degree} degrees")
            
            # Convert to radians and create twist message
            angular_speed_radians = math.radians(angular_speed_degree)
            twist_msg = Twist()
            twist_msg.angular.z = angular_speed_radians * (-1 if clockwise else 1)
            
            # Save start orientation
            start_pose = copy.copy(self.pose)
            rotated_angle = 0.0
            
            # Rotate until desired angle is reached
            while rotated_angle < desired_angle_degree:
                # Publish velocity command
                self.velocity_publisher.publish(twist_msg)
                
                # Calculate rotated angle
                current_angle_diff = abs(self.normalize_angle(start_pose.theta - self.pose.theta))
                rotated_angle = math.degrees(current_angle_diff)
                
                logger.debug(f"Angle rotated: {rotated_angle:.2f}/{desired_angle_degree:.2f} degrees")
                
                # Short sleep to prevent CPU overuse
                time.sleep(0.01)
            
            # Stop the turtle
            twist_msg.angular.z = 0.0
            self.velocity_publisher.publish(twist_msg)
            
            # Final angle rotated
            final_angle = math.degrees(abs(self.normalize_angle(start_pose.theta - self.pose.theta)))
            logger.info(f"Rotation complete. Angle rotated: {final_angle:.2f} degrees")
            
        except Exception as e:
            logger.error(f"Error during rotation operation: {str(e)}")
        finally:
            # Ensure we stop the turtle even if an error occurs
            stop_msg = Twist()
            self.velocity_publisher.publish(stop_msg)
            self.is_rotating = False
    
    def normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def go_to_goal(self, location):
        """Move the turtle to a specified location by name."""
        # Map of named locations to coordinates (x, y)
        locations = {
            "center": (5.5, 5.5),
            "top-left": (1.0, 9.0),
            "top-right": (9.0, 9.0),
            "bottom-left": (1.0, 1.0),
            "bottom-right": (9.0, 1.0),
            "kitchen": (7.0, 3.0),
            "bedroom": (3.0, 7.0),
            "living room": (5.0, 3.0),
            "bathroom": (8.0, 8.0),
        }
        
        try:
            self.is_moving = True
            
            if location.lower() not in locations:
                logger.warning(f"Unknown location: {location}. Going to center instead.")
                x_goal, y_goal = locations["center"]
            else:
                x_goal, y_goal = locations[location.lower()]
            
            logger.info(f"Moving to location '{location}' at coordinates ({x_goal}, {y_goal})")
            
            # Create a twist message for movement
            twist_msg = Twist()
            
            # PID controller parameters
            Kp_linear = 0.5
            Kp_angular = 4.0
            distance_tolerance = 0.1
            
            # Main control loop
            while True:
                # Calculate distance to goal
                distance_to_goal = math.sqrt((x_goal - self.x)**2 + (y_goal - self.y)**2)
                
                # Break if we've reached the goal
                if distance_to_goal < distance_tolerance:
                    break
                
                # Calculate required heading
                desired_angle = math.atan2(y_goal - self.y, x_goal - self.x)
                
                # Calculate the error in heading
                angle_error = self.normalize_angle(desired_angle - self.theta)
                
                # Set the linear and angular velocities proportional to the errors
                twist_msg.linear.x = Kp_linear * distance_to_goal
                twist_msg.angular.z = Kp_angular * angle_error
                
                # Cap the speeds
                if twist_msg.linear.x > 1.0:
                    twist_msg.linear.x = 1.0
                    
                # Publish the velocity command
                self.velocity_publisher.publish(twist_msg)
                
                # Let callbacks process
                self.move_executor.spin_once(timeout_sec=0.1)
                
                # Log progress occasionally
                logger.debug(f"Distance to goal: {distance_to_goal:.2f}, Angle error: {math.degrees(angle_error):.2f} degrees")
            
            # Stop the turtle
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.velocity_publisher.publish(twist_msg)
            
            logger.info(f"Reached location '{location}' at ({self.x:.2f}, {self.y:.2f})")
            
        except Exception as e:
            logger.error(f"Error during go_to_goal operation: {str(e)}")
        finally:
            # Ensure we stop the turtle even if an error occurs
            stop_msg = Twist()
            self.velocity_publisher.publish(stop_msg)
            self.is_moving = False
    
    def execute_sequence(self, actions):
        """Execute a sequence of actions."""
        logger.info(f"Executing sequence of {len(actions)} actions")
        
        for i, action in enumerate(actions):
            logger.info(f"Sequence step {i+1}/{len(actions)}: {action['action']}")
            
            try:
                if action['action'] == 'move':
                    params = action['params']
                    self.move(
                        params.get('linear_speed', 0.2),
                        params.get('distance', 1.0),
                        params.get('is_forward', True)
                    )
                elif action['action'] == 'rotate':
                    params = action['params']
                    self.rotate(
                        params.get('angular_velocity', 10.0),
                        params.get('angle', 90.0),
                        params.get('is_clockwise', True)
                    )
                elif action['action'] == 'go_to_goal':
                    location = action['params']['location']['value']
                    self.go_to_goal(location)
                elif action['action'] == 'stop':
                    logger.info("Stopping the turtle")
                    stop_msg = Twist()
                    self.velocity_publisher.publish(stop_msg)
                    time.sleep(1.0)  # Pause for a moment
                else:
                    logger.warning(f"Unknown action in sequence: {action['action']}")
            except Exception as e:
                logger.error(f"Error executing action {action['action']} in sequence: {str(e)}")
        
        logger.info("Sequence execution complete")

def main(args=None):
    """Main function to start the turtlesim controller node."""
    try:
        # Initialize ROS
        rclpy.init(args=args)
        
        # Create and spin the node
        node = TurtlesimController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
    finally:
        # Clean shutdown
        rclpy.shutdown()
        logger.info("Turtlesim controller shutdown complete")

if __name__ == '__main__':
    main()