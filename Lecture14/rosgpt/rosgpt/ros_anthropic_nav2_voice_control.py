#!/usr/bin/env python3
# This file is part of rosanthropic package.
#
# Copyright (c) 2023-2025 Anis Koubaa.
# Modified for Anthropic Claude AI integration with voice commands.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import rclpy
from rclpy.node import Node
import logging
import threading
import queue
import sounddevice as sd
import numpy as np
import requests
import json
import time
from std_msgs.msg import String

# For VAD (Voice Activity Detection)
import webrtcvad

# For speech recognition
import speech_recognition as sr

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger('rosanthropic_voice')

class AudioProcessor:
    """Processes audio data and detects speech using VAD."""
    
    def __init__(self, sample_rate=16000, frame_duration_ms=30, aggressiveness=3):
        self.sample_rate = sample_rate
        self.frame_duration_ms = frame_duration_ms
        self.vad = webrtcvad.Vad(aggressiveness)
        self.frame_size = int(sample_rate * frame_duration_ms / 1000)
        self.talking_detected = False
        self.silence_frames = 0
        self.talking_frames = 0
        self.max_silence_frames = 30  # About 1 second of silence
        self.min_talking_frames = 10  # Require some talking to count
        
    def is_speech(self, audio_frame):
        """Check if a frame contains speech."""
        try:
            return self.vad.is_speech(audio_frame, self.sample_rate)
        except:
            return False
    
    def process_audio(self, audio_chunk):
        """Process an audio chunk and determine if it contains meaningful speech."""
        # Convert float32 to int16
        audio_data = (audio_chunk * 32767).astype(np.int16).tobytes()
        
        # Process frames
        for i in range(0, len(audio_data) - self.frame_size, self.frame_size):
            frame = audio_data[i:i+self.frame_size]
            if len(frame) != self.frame_size:
                continue
                
            is_speech = self.is_speech(frame)
            
            if is_speech:
                self.talking_frames += 1
                self.silence_frames = 0
            else:
                self.silence_frames += 1
                
            # State machine for detecting talking
            if not self.talking_detected and self.talking_frames > self.min_talking_frames:
                self.talking_detected = True
                self.talking_frames = 0
                return "START_TALKING"
            
            if self.talking_detected and self.silence_frames > self.max_silence_frames:
                self.talking_detected = False
                self.silence_frames = 0
                self.talking_frames = 0
                return "STOP_TALKING"
                
        return "CONTINUE"

class ROSAnthropicVoiceNode(Node):
    """A ROS2 node that listens for voice commands and sends them to the ROSAnthropic server."""
    
    def __init__(self):
        """Initialize the ROSAnthropicVoiceNode."""
        super().__init__('rosanthropic_voice')
        
        # ROS2 parameters
        self.declare_parameter('server_url', 'http://localhost:5000/rosgpt')
        self.server_url = self.get_parameter('server_url').value
        self.declare_parameter('device_index', None)  # None means default microphone
        self.device_index = self.get_parameter('device_index').value
        self.declare_parameter('sample_rate', 16000)
        self.sample_rate = self.get_parameter('sample_rate').value
        
        # Create a publisher for text commands (optional, for debugging)
        self.publisher = self.create_publisher(String, 'text_cmd', 10)
        
        # Set up audio parameters
        self.channels = 1
        self.dtype = 'float32'
        
        # Set up audio processor for VAD
        self.audio_processor = AudioProcessor(sample_rate=self.sample_rate)
        
        # Set up speech recognizer
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000  # Adjust this based on your environment
        
        # Audio buffer and processing flags
        self.audio_buffer = []
        self.recording = False
        self.audio_queue = queue.Queue()
        self.recording_thread = None
        
        # Print available audio devices for the user
        self._print_audio_devices()
        
        # Start listening for audio
        self.start_audio_stream()
        
        self.get_logger().info("ROSAnthropic Voice Node started. Listening for commands...")
    
    def _print_audio_devices(self):
        """Print available audio input devices."""
        try:
            devices = sd.query_devices()
            self.get_logger().info("Available audio input devices:")
            for i, device in enumerate(devices):
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"  [{i}] {device['name']}")
        except Exception as e:
            self.get_logger().error(f"Error querying audio devices: {e}")
    
    def start_audio_stream(self):
        """Start the audio stream to listen for voice commands."""
        try:
            # Start audio callback
            self.stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype=self.dtype,
                callback=self.audio_callback,
                device=self.device_index
            )
            self.stream.start()
            
            # Start processing thread
            self.processing_thread = threading.Thread(target=self.process_audio_thread)
            self.processing_thread.daemon = True
            self.processing_thread.start()
            
            self.get_logger().info("Audio stream started. Listening for commands...")
        except Exception as e:
            self.get_logger().error(f"Error starting audio stream: {e}")
    
    def audio_callback(self, indata, frames, time, status):
        """Callback for the audio stream."""
        if status:
            self.get_logger().warning(f"Audio callback status: {status}")
        
        # Put the audio chunk in the queue for processing
        self.audio_queue.put(indata.copy())
    
    def process_audio_thread(self):
        """Process audio chunks from the queue."""
        while rclpy.ok():
            try:
                # Get audio chunk from the queue
                audio_chunk = self.audio_queue.get(timeout=1.0)
                
                # Process the audio chunk for speech detection
                result = self.audio_processor.process_audio(audio_chunk)
                
                if result == "START_TALKING" and not self.recording:
                    self.get_logger().info("Voice detected, recording...")
                    self.recording = True
                    self.audio_buffer = [audio_chunk]
                
                elif result == "STOP_TALKING" and self.recording:
                    self.get_logger().info("End of speech detected, processing command...")
                    self.recording = False
                    
                    # Process the complete utterance in a separate thread
                    threading.Thread(target=self.process_utterance, 
                                      args=(np.concatenate(self.audio_buffer),)).start()
                    self.audio_buffer = []
                
                elif self.recording:
                    self.audio_buffer.append(audio_chunk)
                
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f"Error in audio processing thread: {e}")
    
    def process_utterance(self, audio_data):
        """Process a complete utterance and send it to the ROSAnthropic server."""
        try:
            # Convert numpy array to AudioData
            audio_int16 = (audio_data * 32767).astype(np.int16)
            audio_bytes = audio_int16.tobytes()
            audio_data_obj = sr.AudioData(audio_bytes, self.sample_rate, 2)
            
            # Recognize speech
            try:
                text = self.recognizer.recognize_google(audio_data_obj)
                self.get_logger().info(f"Recognized: {text}")
                
                # Publish the recognized text (for debugging)
                msg = String()
                msg.data = text
                self.publisher.publish(msg)
                
                # Send the text to the ROSAnthropic server
                self.send_command_to_server(text)
                
            except sr.UnknownValueError:
                self.get_logger().info("Speech Recognition could not understand audio")
            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results from Speech Recognition service; {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing utterance: {e}")
    
    def send_command_to_server(self, text_command):
        """Send the recognized text command to the ROSAnthropic server."""
        try:
            # Prepare data for POST request
            data = {'text_command': text_command}
            
            # Send request with error handling
            self.get_logger().info(f'Sending command to server: "{text_command}"')
            response = requests.post(self.server_url, data=data, timeout=30)
            
            if response.status_code == 200:
                try:
                    # Parse the response
                    response_dict = json.loads(response.content.decode('utf-8'))
                    self.get_logger().info(f"Server response received: {str(response_dict)[:60]}...")
                except Exception as e:
                    self.get_logger().error(f"Error parsing server response: {e}")
            else:
                self.get_logger().error(f"Server error: {response.status_code}")
                
        except requests.exceptions.Timeout:
            self.get_logger().error("Request to server timed out")
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f"Connection error. Server URL: {self.server_url}")
        except Exception as e:
            self.get_logger().error(f"Error sending command to server: {e}")

def main(args=None):
    """Main function to initialize and run the ROSAnthropicVoiceNode."""
    try:
        # Initialize ROS
        rclpy.init(args=args)
        
        # Create and spin the node
        node = ROSAnthropicVoiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown requested by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Clean shutdown
        rclpy.shutdown()
        print("ROSAnthropic Voice Node shutdown complete")

if __name__ == '__main__':
    main()