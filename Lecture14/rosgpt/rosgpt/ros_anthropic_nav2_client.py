#!/usr/bin/env python3
# This file is part of rosanthropic package.
#
# Copyright (c) 2023-2025 Anis Koubaa.
# Modified for Anthropic Claude AI integration with Nav2.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import json
import rclpy
import logging
from rclpy.node import Node
import requests

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger('rosanthropic_nav2_client')

class ROSAnthropicNav2Client(Node):
    def __init__(self):
        """Initialize the ROSAnthropic Nav2 client node."""
        super().__init__('rosanthropic_nav2_client')
        
        # Setup the server URL parameter (defaults to localhost)
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value
        
        # Log startup information
        self.get_logger().info('ROSAnthropic Nav2 client node started')
        self.get_logger().info(f'Connected to server: {self.server_url}')
        
        # Start command processing
        self.send_text_command()

    def send_text_command(self):
        """Sends a text command to the ROSAnthropic system and receives a response from Claude AI."""
        while rclpy.ok():
            print('\n' + '='*50)
            print('ROSANTHROPIC NAV2 COMMANDER')
            print('='*50)
            print('Enter a command to navigate the robot.')
            print('Examples:')
            print('- "Go to top_right_corner"')
            print('- "Navigate to top_h"')
            print('- "Go to top_right_corner, then to top_left_corner, and finally to top_h" (stops at each point)')
            print('- "Follow a continuous path through the top_left_corner, top_right_corner, and bottom_h" (no stopping)')
            print('- "Go to top_h and directly back to bottom_h without stopping"')
            print('-'*50)
            
            # Get user input
            text_command = input("Enter command: ")
            
            if not text_command.strip():
                print("Command cannot be empty. Please try again.")
                continue
                
            # Prepare data for POST request
            data = {'text_command': text_command}
            
            # Send request with error handling
            try:
                self.get_logger().info(f'Sending command: "{text_command}"')
                response = requests.post(self.server_url, data=data, timeout=30)
                
                if response.status_code == 200:
                    try:
                        # Parse the response
                        response_str = response.content.decode('utf-8')
                        response_dict = json.loads(response_str)
                        
                        # Extract and display information
                        print('\nResponse from Claude:')
                        print('-'*50)
                        if 'text' in response_dict:
                            text_response = response_dict['text']
                            self.get_logger().info('Text response received')
                            print(f"Claude's interpretation: {text_response}")
                        
                        if 'json' in response_dict:
                            json_str = response_dict['json']
                            try:
                                json_response = json.loads(json_str)
                                self.get_logger().info(f'JSON response: {json.dumps(json_response, indent=2)}')
                                print("\nStructured command:")
                                print(json.dumps(json_response, indent=2))
                                
                                # Additional info about the action
                                if 'error' in json_response:
                                    print(f"\nError: {json_response['error']}")
                                elif 'action' in json_response:
                                    action = json_response['action']
                                    print(f"\nAction: {action.upper()}")
                                    
                                    if action == 'go_to_goal':
                                        location = json_response['params']['location']['value']
                                        print(f"Robot will navigate to: {location}")
                                    
                                    elif action == 'follow_path':
                                        print("Robot will follow a continuous path through these locations:")
                                        for i, loc in enumerate(json_response['params']['locations']):
                                            print(f"  {i+1}. {loc['value']}")
                                        print("(Robot will not stop at intermediate locations)")
                                    
                                    elif action == 'sequence':
                                        continuous = json_response.get('continuous_path', False)
                                        path_type = "continuous path" if continuous else "waypoint sequence"
                                        print(f"Robot will follow a {path_type} through these locations:")
                                        for i, step in enumerate(json_response['params']):
                                            if 'action' in step and step['action'] == 'go_to_goal':
                                                loc = step['params']['location']['value']
                                                print(f"  {i+1}. {loc}")
                                        if continuous:
                                            print("(Robot will not stop at intermediate locations)")
                                        else:
                                            print("(Robot will stop at each location)")
                                
                            except json.JSONDecodeError:
                                self.get_logger().error(f'Invalid JSON in response: {json_str}')
                                print("Error: Could not parse the JSON response")
                            
                    except Exception as e:
                        self.get_logger().error(f'Error processing response: {str(e)}')
                        print(f"Error processing response: {str(e)}")
                else:
                    self.get_logger().error(f'Server error: {response.status_code}')
                    print(f"Server returned error: {response.status_code}")
                    if response.content:
                        try:
                            error_data = response.json()
                            print(f"Error details: {error_data.get('error', 'Unknown error')}")
                        except:
                            print(f"Error content: {response.content.decode('utf-8')}")
            
            except requests.exceptions.Timeout:
                self.get_logger().error('Request timed out')
                print("Request timed out. Server may be busy or unavailable.")
            
            except requests.exceptions.ConnectionError:
                self.get_logger().error('Connection error')
                print("Connection error. Please check if the server is running.")
                print(f"Server URL: {self.server_url}")
                
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {str(e)}')
                print(f"Unexpected error: {str(e)}")
            
            print('\n' + '-'*50)
            input("Press Enter to continue...")


def main(args=None):
    """Main function to initialize and run the ROSAnthropic Nav2 client node."""
    try:
        # Initialize ROS
        rclpy.init(args=args)
        
        # Create and spin the client node
        rosanthropic_client = ROSAnthropicNav2Client()
        rclpy.spin(rosanthropic_client)
        
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        # Clean shutdown
        if 'rosanthropic_client' in locals():
            rosanthropic_client.destroy_node()
        rclpy.shutdown()
        print("ROSAnthropic Nav2 client shutdown complete")


if __name__ == '__main__':
    main()