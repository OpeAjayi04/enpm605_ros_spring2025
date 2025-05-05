#!/usr/bin/env python3
# This file is part of rosgpt package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import os
import sys
import json
import logging
import threading
import subprocess
from contextlib import contextmanager
from typing import Optional, Dict, Any, Union, List

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger('rosgpt')

# Try importing required packages with helpful error messages
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.executors import SingleThreadedExecutor
except ImportError as e:
    logger.error(f"ROS2 package import error: {e}")
    logger.error("Make sure ROS2 is installed and properly sourced")
    sys.exit(1)

try:
    import openai
except ImportError:
    logger.error("OpenAI package not found. Install with: pip install openai")
    sys.exit(1)

try:
    from flask import Flask, request, send_from_directory, jsonify
    from flask_restful import Resource, Api
    from flask_cors import CORS
except ImportError:
    logger.error("Flask packages not found. Install with: pip install flask flask-restful flask-cors")
    sys.exit(1)

# Check OpenAI package version and import accordingly
try:
    import pkg_resources
    openai_version = pkg_resources.get_distribution("openai").version
    logger.info(f"Detected OpenAI package version: {openai_version}")
    # Parse major version to adjust API usage
    major_version = int(openai_version.split('.')[0])
except Exception as e:
    logger.warning(f"Could not determine OpenAI package version: {e}")
    major_version = 0  # Default to older version handling

# Import pyttsx3 with fallback to avoid TTS errors breaking the whole system
tts_available = True
try:
    import pyttsx3
except ImportError:
    logger.warning("pyttsx3 not installed. Text-to-speech will be disabled.")
    logger.warning("Install with: pip install pyttsx3 and sudo apt-get install libespeak1")
    tts_available = False

# Instantiate a Flask application object
app = Flask(__name__)

# Enable Cross-Origin Resource Sharing (CORS) for the Flask app
CORS(app)

# Create an API object that wraps the Flask app to handle RESTful requests
api = Api(app)

# Get the API key from the environment variable with validation
openai_api_key = os.getenv('OPENAI_API_KEY')
if not openai_api_key:
    logger.error("OPENAI_API_KEY environment variable not set.")
    logger.error("Set it with: export OPENAI_API_KEY=your_api_key")
    sys.exit(1)

# Configure the OpenAI API with the provided key
openai.api_key = openai_api_key

# Initialize threading locks for synchronizing access
spin_lock = threading.Lock()
tts_lock = threading.Lock()

# Initialize the Text-to-Speech engine if available
tts_engine = None
if tts_available:
    try:
        tts_engine = pyttsx3.init()
        logger.info("Text-to-speech engine initialized successfully")
    except Exception as e:
        logger.warning(f"Failed to initialize text-to-speech engine: {e}")
        logger.warning("Text-to-speech functionality will be disabled")
        tts_available = False


def speak(text: str) -> None:
    """
    Uses the Text-to-Speech engine to speak the given text.
    Falls back gracefully if TTS is not available.

    Args:
        text (str): The text to be spoken by the TTS engine.
    """
    if not tts_available or tts_engine is None:
        logger.debug(f"TTS disabled. Would have said: {text}")
        return

    try:
        with tts_lock:
            tts_engine.say(text)
            tts_engine.runAndWait()
    except Exception as e:
        logger.warning(f"Text-to-speech error: {e}")


class ROSGPTNode(Node):
    """A ROS2 node for interfacing with the OpenAI API."""
    
    def __init__(self):
        """Initialize the ROSGPTNode with error handling."""
        try:
            # Call the superclass constructor and pass the name of the node
            super().__init__('chatgpt_ros2_node')
            # Create a publisher for the 'voice_cmd' topic with a message queue size of 10
            self.publisher = self.create_publisher(String, 'voice_cmd', 10)
            logger.info("ROSGPTNode initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize ROSGPTNode: {e}")
            raise

    def publish_message(self, message: str) -> None:
        """
        Publish the given message to the 'voice_cmd' topic with error handling.
        
        Args:
            message (str): The message to be published.
        """
        try:
            msg = String()
            msg.data = message
            self.publisher.publish(msg)
            logger.debug(f"Message published: {message[:50]}...")
        except Exception as e:
            logger.error(f"Error publishing message: {e}")


@contextmanager
def safe_executor(node: Node):
    """
    A context manager for safely using SingleThreadedExecutor with proper cleanup.
    
    Args:
        node (Node): The ROS2 node to add to the executor.
        
    Yields:
        SingleThreadedExecutor: The executor with the node added.
    """
    executor = SingleThreadedExecutor()
    try:
        executor.add_node(node)
        yield executor
    finally:
        try:
            executor.remove_node(node)
        except Exception as e:
            logger.warning(f"Error removing node from executor: {e}")


def process_and_publish_chatgpt_response(
    chatgpt_ros2_node: ROSGPTNode, 
    text_command: str, 
    chatgpt_response: str, 
    use_executors: bool = True
) -> None:
    """
    Process the chatbot's response and publish it to the 'voice_cmd' topic with robust error handling.

    Args:
        chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        text_command (str): The text command received from the user.
        chatgpt_response (str): The response from the chatbot.
        use_executors (bool, optional): Flag to indicate whether to use SingleThreadedExecutor. Defaults to True.
    """
    try:
        chatgpt_ros2_node.publish_message(chatgpt_response)
        
        # If use_executors flag is True, use SingleThreadedExecutor with context manager
        if use_executors:
            with safe_executor(chatgpt_ros2_node) as executor:
                executor.spin_once()
        # If use_executors flag is False, use spin_lock to synchronize access
        else:
            with spin_lock:
                rclpy.spin_once(chatgpt_ros2_node)
    except Exception as e:
        logger.error(f"Error processing or publishing ChatGPT response: {e}")


class ROSGPTProxy(Resource):
    """
    A class derived from flask_restful.Resource, responsible for handling incoming HTTP POST requests.
    """

    def __init__(self, chatgpt_ros2_node: ROSGPTNode):
        """
        Initialize the ROSGPTProxy class with the given ROS2 node.

        Args:
            chatgpt_ros2_node (ROSGPTNode): The ROS2 node instance.
        """
        self.chatgpt_ros2_node = chatgpt_ros2_node

    def askGPT(self, text_command: str) -> Optional[str]:
        """
        Send a text command to the GPT-3 model and receive a response with robust error handling.
        
        Args:
            text_command (str): The text command to be sent to the GPT-3 model.
            
        Returns:
            Optional[str]: The response from the GPT-3 model as a JSON string, or None if an error occurs.
        """
        # Create the GPT-3 prompt with example inputs and desired outputs
        prompt = '''Consider the following ontology:
                    {"action": "go_to_goal", "params": {"location": {"type": "str", "value": location}}}
                    {"action": "move", "params": {"linear_speed": linear_speed, "distance": distance, "is_forward": is_forward}}
                    {"action": "rotate", "params": {"angular_velocity": angular_velocity, "angle": angle, "is_clockwise": is_clockwise}}

                    You will be given human language prompts, and you need to return a JSON conformant to the ontology. Any action not in the ontology must be ignored. Here are some examples.

                    prompt: "Move forward for 1 meter at a speed of 0.5 meters per second."
                    returns: {"action": "move", "params": {"linear_speed": 0.5, "distance": 1, "is_forward": true, "unit": "meter"}}

                    prompt: "Rotate 60 degree in clockwise direction at 10 degrees per second and make pizza."
                    returns: {"action": "rotate", "params": {"angular_velocity": 10, "angle": 60, "is_clockwise": true, "unit": "degrees"}}
                    
                    prompt: "go to the bedroom, rotate 60 degrees and move 1 meter then stop"
                    returns: {"action": "sequence", "params": [{"action": "go_to_goal", "params": {"location": {"type": "str", "value": "bedroom"}}}, {"action": "rotate", "params": {"angular_velocity": 30, "angle": 60, "is_clockwise": false, "unit": "degrees"}}, {"action": "move", "params": {"linear_speed": 1, "distance": 1, "is_forward": true, "unit": "meter"}}, {"action": "stop"}]}
                    '''
        prompt = prompt+'\nprompt: '+text_command
        logger.debug(f"Generated prompt: {prompt[:100]}...")
        
        # Create the message structure for the GPT-3 model
        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": prompt}
        ]

        # Try to send the request to the GPT-3 model with comprehensive error handling
        max_retries = 3
        retry_count = 0
        backoff_factor = 2  # seconds
        
        while retry_count < max_retries:
            try:
                # Check network connectivity before making the request
                try:
                    # Simple check - try to resolve openai.com
                    subprocess.check_call(
                        ["ping", "-c", "1", "api.openai.com"], 
                        stdout=subprocess.DEVNULL, 
                        stderr=subprocess.DEVNULL
                    )
                except subprocess.CalledProcessError:
                    logger.warning("Network connectivity issue detected. Cannot reach api.openai.com")
                    return json.dumps({
                        "text": "Error: Network connectivity issue. Cannot reach OpenAI API.",
                        "json": "{\"error\": \"network_connectivity_issue\"}"
                    })
                
                # Make the actual API call based on OpenAI version
                if major_version >= 1:
                    # New OpenAI API (v1.x.x+)
                    client = openai.OpenAI(api_key=openai_api_key)
                    response = client.chat.completions.create(
                        model="gpt-3.5-turbo",
                        messages=messages,
                        timeout=30  # 30 second timeout
                    )
                    # Access response differently in new API
                    chatgpt_response = response.choices[0].message.content.strip()
                else:
                    # Legacy OpenAI API (v0.x.x)
                    response = openai.ChatCompletion.create(
                        model="gpt-3.5-turbo",
                        messages=messages,
                        timeout=30  # 30 second timeout
                    )
                    # Extract the GPT-3 model response from the returned JSON
                    chatgpt_response = response.choices[0].message['content'].strip()
                
                # Find the start and end indices of the JSON string in the response
                start_index = chatgpt_response.find('{')
                end_index = chatgpt_response.rfind('}') + 1
                
                # Check if valid JSON was found
                if start_index == -1 or end_index <= 0 or start_index >= end_index:
                    logger.warning("No valid JSON found in ChatGPT response")
                    json_response_dict = "{\"error\": \"no_valid_json_found\"}"
                else:
                    # Extract the JSON string from the response
                    json_response_dict = chatgpt_response[start_index:end_index]
                    # Validate JSON
                    try:
                        json.loads(json_response_dict)
                    except json.JSONDecodeError:
                        logger.warning(f"Invalid JSON in response: {json_response_dict}")
                        json_response_dict = "{\"error\": \"invalid_json_format\"}"
                
                return json.dumps({'text': chatgpt_response, 'json': json_response_dict})
            
            except Exception as e:
                error_type = type(e).__name__
                error_message = str(e)
                
                # Handle different error types based on error messages and class names
                if "timeout" in error_message.lower() or error_type == "Timeout":
                    logger.warning(f"OpenAI API timeout (attempt {retry_count+1}/{max_retries}): {e}")
                    if retry_count == max_retries - 1:
                        return json.dumps({
                            "text": "Error: OpenAI API request timed out after multiple attempts.",
                            "json": "{\"error\": \"openai_api_timeout\"}"
                        })
                
                elif "connection" in error_message.lower() or error_type == "APIConnectionError":
                    logger.warning(f"OpenAI API connection error (attempt {retry_count+1}/{max_retries}): {e}")
                    if retry_count == max_retries - 1:
                        return json.dumps({
                            "text": "Error: Unable to connect to OpenAI API. Please check your network connection and proxy settings.",
                            "json": "{\"error\": \"openai_api_connection_error\"}"
                        })
                
                elif "api" in error_type.lower() and "error" in error_type.lower():
                    logger.warning(f"OpenAI API error (attempt {retry_count+1}/{max_retries}): {e}")
                    if retry_count == max_retries - 1:
                        return json.dumps({
                            "text": "Error: OpenAI API returned an error. Please try again later.",
                            "json": "{\"error\": \"openai_api_error\"}"
                        })
                
                elif "auth" in error_message.lower() or "key" in error_message.lower() or error_type == "AuthenticationError":
                    logger.error(f"OpenAI API authentication error: {e}")
                    return json.dumps({
                        "text": "Error: OpenAI API authentication failed. Please check your API key.",
                        "json": "{\"error\": \"openai_api_authentication_error\"}"
                    })
                
                elif "rate" in error_message.lower() or "limit" in error_message.lower() or error_type == "RateLimitError":
                    logger.warning(f"OpenAI API rate limit error: {e}")
                    # For rate limit errors, always retry with a longer backoff
                    if retry_count < max_retries - 1:
                        # Exponential backoff with jitter for rate limits
                        import random
                        base_sleep = backoff_factor ** (retry_count + 2)  # Longer backoff for rate limits
                        jitter = random.uniform(0, 1)
                        sleep_time = base_sleep + jitter
                        logger.info(f"Rate limit exceeded. Retrying in {sleep_time:.2f} seconds...")
                        threading.Event().wait(sleep_time)
                        continue  # Skip to next retry immediately
                    else:
                        return json.dumps({
                            "text": "Error: OpenAI API rate limit exceeded. Please try again in a few minutes.",
                            "json": "{\"error\": \"openai_api_rate_limit_error\"}"
                        })
                
                elif "invalid" in error_message.lower() or "request" in error_message.lower() or error_type == "InvalidRequestError":
                    logger.error(f"OpenAI API invalid request error: {e}")
                    return json.dumps({
                        "text": "Error: Invalid request to OpenAI API. Please check your prompt and parameters.",
                        "json": "{\"error\": \"openai_api_invalid_request_error\"}"
                    })
                
                else:
                    logger.error(f"Unexpected error communicating with OpenAI API (attempt {retry_count+1}/{max_retries}): {e}")
                    if retry_count == max_retries - 1:
                        return json.dumps({
                            "text": f"Error: An unexpected error occurred: {error_type} - {error_message}",
                            "json": "{{\"error\": \"unexpected_error\", \"details\": \"{0}\"}}".format(error_type)
                        })
            
            # Increase retry count and sleep before retrying
            retry_count += 1
            if retry_count < max_retries:
                sleep_time = backoff_factor ** retry_count
                logger.info(f"Retrying in {sleep_time} seconds...")
                threading.Event().wait(sleep_time)
        
        # This should never be reached due to the returns in the exception handlers
        return None

    def post(self):
        """
        Handles an incoming POST request containing a text command with improved error handling.
        
        Returns:
            dict: A dictionary containing the GPT-3 model response as a JSON string or error information.
        """
        try:
            if not request.form or 'text_command' not in request.form:
                logger.warning("Invalid POST request: 'text_command' field missing")
                return {'error': 'Missing text_command field'}, 400
            
            text_command = request.form['text_command']
            if not text_command or not text_command.strip():
                logger.warning("Empty text command received")
                return {'error': 'Empty text command'}, 400
            
            logger.info(f"[ROSGPT] Command received: {text_command}")
            
            # Acknowledge receipt on a separate thread
            threading.Thread(
                target=speak, 
                args=(f"{text_command}. Message received. Now consulting ChatGPT for a response.",)
            ).start()
            
            # Get response from ChatGPT
            chatgpt_response = self.askGPT(text_command)
            
            if chatgpt_response is None:
                logger.error("Failed to get response from ChatGPT")
                return {'error': 'Failed to get response from ChatGPT'}, 500
            
            try:
                response_obj = json.loads(chatgpt_response)
                logger.info(f"[ROSGPT] Response received from ChatGPT: {str(response_obj)[:60]}...")
            except json.JSONDecodeError as e:
                logger.error(f"Invalid JSON response from askGPT: {e}")
                return {'error': 'Invalid JSON response from ChatGPT'}, 500
            
            # Notify about the received response
            threading.Thread(
                target=speak, 
                args=("We have received a response from ChatGPT.",)
            ).start()
            
            # Process and publish the response in a separate thread
            threading.Thread(
                target=process_and_publish_chatgpt_response, 
                args=(self.chatgpt_ros2_node, text_command, chatgpt_response, True)
            ).start()
            
            return response_obj
            
        except Exception as e:
            logger.error(f"Unexpected error handling POST request: {e}")
            return {'error': f'An unexpected error occurred: {str(e)}'}, 500


@app.route('/')
def index():
    """Serve the index.html file from the webapp directory."""
    try:
        webapp_path = os.path.join(app.root_path, 'webapp')
        if not os.path.exists(webapp_path):
            logger.warning(f"Webapp directory not found: {webapp_path}")
            return "Webapp directory not found", 404
        
        index_path = os.path.join(webapp_path, 'index.html')
        if not os.path.exists(index_path):
            logger.warning(f"Index.html not found: {index_path}")
            return "Index.html not found", 404
        
        return send_from_directory(webapp_path, 'index.html')
    except Exception as e:
        logger.error(f"Error serving index.html: {e}")
        return f"Error: {str(e)}", 500


def check_dependencies():
    """Check if all required dependencies are installed and configured."""
    missing_deps = []
    
    # Check ROS2 modules
    try:
        import rclpy
    except ImportError:
        missing_deps.append("ROS2 Python modules (rclpy)")
    
    # Check OpenAI
    try:
        import openai
    except ImportError:
        missing_deps.append("openai")
    
    # Check Flask and related
    try:
        import flask
        import flask_restful
        import flask_cors
    except ImportError:
        missing_deps.append("Flask packages (flask, flask-restful, flask-cors)")
    
    # Check pyttsx3 (optional)
    try:
        import pyttsx3
    except ImportError:
        logger.warning("pyttsx3 not installed. Text-to-speech will be disabled.")
    
    # Report missing dependencies
    if missing_deps:
        logger.error("Missing dependencies detected:")
        for dep in missing_deps:
            logger.error(f"  - {dep}")
        logger.error("Please install missing dependencies before continuing.")
        return False
    
    # Check API key
    if not os.getenv('OPENAI_API_KEY'):
        logger.error("OPENAI_API_KEY environment variable not set.")
        logger.error("Set it with: export OPENAI_API_KEY=your_api_key")
        return False
    
    return True


def main():
    """Main function to initialize and run the ROSGPT node."""
    try:
        # Check dependencies before starting
        if not check_dependencies():
            sys.exit(1)
        
        # Initialize ROS2
        rclpy.init(args=None)
        
        # Create ROS2 node
        chatgpt_ros2_node = ROSGPTNode()
        
        # Add resource to API
        api.add_resource(
            ROSGPTProxy, 
            '/rosgpt', 
            resource_class_args=(chatgpt_ros2_node,)
        )
        
        # Log startup info
        logger.info("ROSGPT node initialized successfully")
        logger.info("Starting Flask server on 0.0.0.0:5000")
        
        # Run Flask app
        app.run(debug=True, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    except Exception as e:
        logger.error(f"Unexpected error in main: {e}")
    finally:
        try:
            rclpy.shutdown()
            logger.info("ROS2 shutdown complete")
        except Exception as e:
            logger.error(f"Error during ROS2 shutdown: {e}")


if __name__ == '__main__':
    main()