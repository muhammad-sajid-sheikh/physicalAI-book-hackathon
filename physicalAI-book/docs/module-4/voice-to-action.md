---
title: Voice-to-Action Processing
sidebar_position: 2
---

# Voice-to-Action Processing

This chapter covers speech recognition fundamentals and voice command processing for humanoid robotics, providing students with the knowledge to implement voice command processing systems that can interact with humanoid robots through ROS 2 action servers.

## Learning Objectives

After completing this chapter, students will be able to:
- Understand speech recognition techniques for robotic applications
- Implement voice command processing pipelines
- Integrate voice commands with ROS 2 action servers
- Handle error cases and validation in voice processing

## Prerequisites

Students should have:
- Basic understanding of speech recognition concepts
- Experience with ROS 2 action servers
- Knowledge of audio processing fundamentals

## Introduction to Speech Recognition in Robotics

Speech recognition technology has become a critical component in human-robot interaction, enabling more natural and intuitive communication between humans and robots. In the context of humanoid robotics, voice commands provide a high-level interface that abstracts the complexity of direct robot control, allowing users to express their intentions in natural language.

The voice-to-action pipeline involves several key stages:
1. Audio capture and preprocessing
2. Speech-to-text conversion
3. Natural language understanding
4. Intent extraction and validation
5. Action mapping to robot capabilities
6. Execution through ROS 2 action servers

This chapter will explore each of these stages in detail, providing practical examples for implementation in humanoid robotics applications.

## Speech Recognition Fundamentals

### Audio Capture and Preprocessing

The first step in any voice command system is capturing high-quality audio. In robotic applications, this involves several challenges that are not present in traditional voice recognition systems:

- **Environmental noise**: Robots operate in various environments with different noise profiles
- **Distance and direction**: The microphone may be distant from the speaker
- **Robot self-noise**: Motors, fans, and other robot components create background noise
- **Acoustic reflections**: Room acoustics can affect audio quality

To address these challenges, consider implementing:

1. **Noise reduction algorithms**: Use digital signal processing techniques to filter out background noise
2. **Beamforming**: If using multiple microphones, focus on the direction of the speaker
3. **Automatic gain control**: Adjust sensitivity based on environmental conditions
4. **Voice activity detection**: Only process audio when speech is detected

### Speech-to-Text Conversion

Modern speech-to-text systems can be implemented using various approaches:

1. **Cloud-based services** (e.g., Google Speech-to-Text, AWS Transcribe, Azure Speech Services)
   - Pros: High accuracy, low latency, multilingual support
   - Cons: Requires internet connection, potential privacy concerns, cost per request

2. **On-device models** (e.g., Vosk, SpeechRecognition library with CMU Sphinx)
   - Pros: No internet required, privacy-preserving, no per-request costs
   - Cons: Lower accuracy, higher computational requirements, limited language support

3. **Custom-trained models** (e.g., using TensorFlow, PyTorch)
   - Pros: Can be optimized for specific domains and vocabularies
   - Cons: Requires significant training data and expertise

For humanoid robotics applications, the choice depends on operational requirements:
- Online systems for high-accuracy applications with reliable connectivity
- Offline systems for privacy-sensitive or connectivity-limited environments
- Custom models for domain-specific vocabulary and commands

## Intent Extraction and Natural Language Understanding

Once speech is converted to text, the system must extract the user's intent. This involves:

1. **Command identification**: Recognizing that a command is being issued
2. **Entity extraction**: Identifying specific objects, locations, or parameters
3. **Action mapping**: Translating the intent to specific robot capabilities

### Example Intent Extraction

Consider the command: "Robot, please move to the kitchen and pick up the red cup."

The intent extraction process would identify:
- **Action**: Move and pick up
- **Destination**: Kitchen
- **Object**: Red cup
- **Sequence**: Navigate first, then manipulate

## Integration with ROS 2 Action Servers

ROS 2 provides a robust framework for implementing action-based robot behaviors. The voice command processing system should interface with existing action servers through a command interpretation layer.

### Architecture Overview

```
Voice Input → Speech-to-Text → NLU → Command Mapping → ROS 2 Actions
```

### Command Mapping Strategy

A command mapping system should:
1. Validate commands against robot capabilities
2. Generate appropriate action goals
3. Handle sequential actions when needed
4. Provide feedback to the user

### Example Implementation

Here's a basic example of how voice commands might be mapped to ROS 2 actions:

```python
class VoiceCommandInterpreter:
    def __init__(self):
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, MoveItGoal, 'move_group')

    def interpret_command(self, text):
        # Parse the command text
        tokens = text.lower().split()

        if 'move' in tokens or 'go' in tokens:
            return self.handle_navigation_command(tokens)
        elif 'pick' in tokens or 'grasp' in tokens:
            return self.handle_manipulation_command(tokens)
        else:
            return self.handle_unknown_command(text)
```

## Error Handling and Validation

Robust voice command systems must handle various error conditions:

1. **Speech recognition errors**: When the speech-to-text system produces incorrect output
2. **Intent ambiguity**: When the command is unclear or ambiguous
3. **Capability mismatch**: When the requested action exceeds robot capabilities
4. **Safety validation**: Ensuring commands don't result in unsafe robot behavior

### Safety Validation Approaches

- **Pre-execution checks**: Validate commands before execution
- **Runtime monitoring**: Monitor robot behavior during execution
- **Emergency stop**: Provide mechanisms to interrupt dangerous actions
- **Command confirmation**: For critical commands, request user confirmation

## Practical Exercise: Implementing a Basic Voice Command System

In this exercise, students will implement a simple voice command system that can handle basic navigation commands for a humanoid robot simulation.

### Requirements:
1. Implement speech-to-text functionality
2. Create a simple intent extractor for navigation commands
3. Interface with a ROS 2 navigation action server
4. Add basic error handling and validation

### Implementation Steps:

1. Set up audio input and speech recognition
2. Create a command parser for navigation intents
3. Map intents to ROS 2 navigation goals
4. Test with simulated robot

## Integration with ROS 2 Action Servers (Detailed)

Now let's look at the detailed implementation of how voice commands can be integrated with ROS 2 action servers for robot control.

### ROS 2 Action Server Interface

ROS 2 actions provide a mechanism for long-running tasks with feedback and status updates. For voice command processing, we typically interact with existing action servers such as:

- **Navigation**: `nav2_msgs/action/NavigateToPose` for navigation tasks
- **Manipulation**: `moveit_msgs/action/MoveGroup` for manipulation tasks
- **Speech**: Custom action servers for text-to-speech feedback

### Example Voice Command Node Implementation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
import speech_recognition as sr
import threading
import json

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set up ROS 2 action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.moveit_client = ActionClient(self, MoveGroup, 'move_group')

        # Publisher for feedback
        self.feedback_pub = self.create_publisher(String, 'voice_feedback', 10)

        # Start listening thread
        self.listening_thread = threading.Thread(target=self.listen_for_commands)
        self.listening_thread.daemon = True
        self.listening_thread.start()

        self.get_logger().info('Voice Command Node initialized')

    def listen_for_commands(self):
        """Continuously listen for voice commands"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info('Listening for commands...')
                    audio = self.recognizer.listen(source, timeout=5.0)

                # Convert speech to text
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Recognized: {text}')

                # Process the command
                self.process_command(text)

            except sr.WaitTimeoutError:
                # No speech detected, continue listening
                continue
            except sr.UnknownValueError:
                self.publish_feedback('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
                self.publish_feedback('Speech recognition service unavailable')

    def process_command(self, text):
        """Process the recognized text and execute appropriate action"""
        text_lower = text.lower()

        if 'move to' in text_lower or 'go to' in text_lower:
            self.handle_navigation_command(text_lower)
        elif 'pick up' in text_lower or 'grasp' in text_lower:
            self.handle_manipulation_command(text_lower)
        elif 'stop' in text_lower or 'halt' in text_lower:
            self.handle_stop_command()
        else:
            self.publish_feedback(f'Unknown command: {text}')

    def handle_navigation_command(self, command):
        """Handle navigation-related voice commands"""
        # Extract destination from command
        destination = self.extract_destination(command)

        if destination:
            # Map destination to coordinates (simplified example)
            pose = self.get_pose_for_location(destination)
            if pose:
                self.send_navigation_goal(pose)
            else:
                self.publish_feedback(f'Unknown destination: {destination}')
        else:
            self.publish_feedback('Could not identify destination')

    def extract_destination(self, command):
        """Extract destination from command text"""
        # Simple keyword-based extraction (in practice, use NLP)
        if 'kitchen' in command:
            return 'kitchen'
        elif 'living room' in command or 'livingroom' in command:
            return 'living_room'
        elif 'bedroom' in command:
            return 'bedroom'
        elif 'office' in command:
            return 'office'
        else:
            return None

    def get_pose_for_location(self, location):
        """Get predefined pose for a location"""
        locations = {
            'kitchen': {'x': 1.0, 'y': 2.0, 'theta': 0.0},
            'living_room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -1.0, 'y': 1.0, 'theta': 1.57},
            'office': {'x': 2.0, 'y': -1.0, 'theta': -1.57}
        }

        if location in locations:
            loc_data = locations[location]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = loc_data['x']
            pose.pose.position.y = loc_data['y']
            pose.pose.position.z = 0.0
            # Simple orientation (facing forward)
            pose.pose.orientation.z = loc_data['theta']
            return pose
        return None

    def send_navigation_goal(self, pose):
        """Send navigation goal to ROS 2 action server"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_callback)

        self.publish_feedback(f'Navigating to location')

    def navigation_goal_callback(self, future):
        """Callback for navigation goal result"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Navigation goal accepted')
                # Wait for result
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.navigation_result_callback)
            else:
                self.get_logger().error('Navigation goal rejected')
                self.publish_feedback('Navigation goal was rejected')
        except Exception as e:
            self.get_logger().error(f'Navigation goal error: {e}')

    def navigation_result_callback(self, future):
        """Callback for navigation result"""
        try:
            result = future.result().result
            self.publish_feedback('Navigation completed successfully')
        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self.publish_feedback('Navigation failed')

    def handle_manipulation_command(self, command):
        """Handle manipulation-related voice commands"""
        # Extract object from command
        obj = self.extract_object(command)

        if obj:
            self.send_manipulation_goal(obj)
        else:
            self.publish_feedback('Could not identify object to manipulate')

    def extract_object(self, command):
        """Extract object from command text"""
        # Simple keyword-based extraction
        if 'red cup' in command or 'red cup' in command:
            return 'red_cup'
        elif 'blue box' in command or 'blue box' in command:
            return 'blue_box'
        elif 'green ball' in command or 'green ball' in command:
            return 'green_ball'
        else:
            return None

    def send_manipulation_goal(self, obj):
        """Send manipulation goal to ROS 2 action server"""
        # This is a simplified example - real implementation would be more complex
        goal_msg = MoveGroup.Goal()
        # Configure the goal based on the object
        self.moveit_client.wait_for_server()

        future = self.moveit_client.send_goal_async(goal_msg)
        future.add_done_callback(self.manipulation_goal_callback)

        self.publish_feedback(f'Attempting to manipulate {obj}')

    def handle_stop_command(self):
        """Handle stop commands"""
        # Cancel any ongoing actions
        self.publish_feedback('Stopping all robot actions')

    def publish_feedback(self, message):
        """Publish feedback to user"""
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)
        self.get_logger().info(f'Feedback: {message}')


def main(args=None):
    rclpy.init(args=args)

    voice_node = VoiceCommandNode()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Code Example: Speech-to-Text with Validation

```python
import speech_recognition as sr
from rclpy.node import Node

class SpeechValidator:
    def __init__(self, node: Node):
        self.node = node
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 4000  # Adjust for environment
        self.recognizer.dynamic_energy_threshold = True

        # Define allowed commands to prevent invalid inputs
        self.allowed_commands = [
            'move', 'go', 'navigate', 'go to', 'move to',
            'pick up', 'grasp', 'take', 'stop', 'halt',
            'help', 'status', 'cancel'
        ]

    def validate_command(self, text):
        """Validate that the command is appropriate for the robot"""
        text_lower = text.lower()

        # Check if command contains known actions
        has_valid_command = any(cmd in text_lower for cmd in self.allowed_commands)

        if not has_valid_command:
            return False, "Command not recognized. Available commands: move, go to, pick up, stop, etc."

        # Additional validation could include:
        # - Checking for required parameters
        # - Verifying destinations are known
        # - Ensuring safety constraints

        return True, "Command is valid"
```

## Error Handling and Validation Approaches

### Speech Recognition Error Handling

Speech recognition systems are prone to various types of errors that need to be handled gracefully:

1. **Audio Quality Issues**: Poor microphone placement, background noise, or low volume can cause recognition failures.

2. **Ambiguous Commands**: Natural language often contains ambiguity that the system must resolve or ask for clarification.

3. **Domain Mismatch**: Commands that are syntactically valid but outside the robot's operational domain.

### Example Error Handling Implementation

```python
class VoiceCommandErrorHandling:
    def __init__(self, node):
        self.node = node
        self.command_history = []
        self.max_history = 10

    def handle_recognition_error(self, error_type, error_message):
        """Handle different types of recognition errors"""
        if error_type == "timeout":
            self.node.get_logger().warn("No speech detected within timeout period")
            self.suggest_commands()
        elif error_type == "unknown":
            self.node.get_logger().info("Could not understand audio")
            self.ask_for_repetition()
        elif error_type == "service_unavailable":
            self.node.get_logger().error("Speech recognition service unavailable")
            self.fallback_to_alternate_input()

    def suggest_commands(self):
        """Suggest valid commands to the user"""
        suggestions = [
            "You can say: 'move to kitchen', 'pick up red cup', 'stop', 'help'",
            "Available locations: kitchen, living room, bedroom, office",
            "Available actions: move, go to, pick up, stop"
        ]
        for suggestion in suggestions:
            self.publish_feedback(suggestion)

    def ask_for_repetition(self):
        """Ask user to repeat the command"""
        self.publish_feedback("I didn't understand. Please repeat your command clearly.")
        self.publish_feedback("Example: 'Please move to the kitchen' or 'Go to the living room'")

    def validate_intent_safety(self, intent):
        """Validate that the intent is safe to execute"""
        # Check if destination is safe
        if intent.get('action') == 'navigate' and not self.is_safe_destination(intent.get('destination')):
            return False, "Destination is unsafe or inaccessible"

        # Check if manipulation is safe
        if intent.get('action') == 'manipulate' and not self.is_safe_manipulation(intent.get('object')):
            return False, "Object manipulation is unsafe"

        return True, "Intent is safe to execute"

    def is_safe_destination(self, destination):
        """Check if destination is safe for navigation"""
        # Implement safety checks based on robot's map and current state
        # This could include checking for obstacles, restricted areas, etc.
        return True  # Simplified for example

    def is_safe_manipulation(self, obj):
        """Check if object manipulation is safe"""
        # Implement safety checks for manipulation
        # This could include checking object properties, robot state, etc.
        return True  # Simplified for example
```

## Accessibility Considerations

When designing voice command systems for humanoid robots, it's important to consider accessibility requirements:

1. **Alternative Input Methods**: Provide alternative ways to interact with the robot for users who cannot use voice commands.

2. **Clear Feedback**: Ensure the robot provides clear audio and visual feedback about its state and actions.

3. **Adjustable Sensitivity**: Allow users to adjust the sensitivity of voice recognition based on their voice characteristics.

4. **Multiple Language Support**: Support multiple languages and accents as appropriate for the deployment environment.

## Sources & Further Reading

1. Hönig, F., et al. (2019). "Speech and Language Technology for Socially Interactive Robots." ACM Computing Surveys.
2. Tapus, A., et al. (2007). "User acceptance of robot companions: A pilot study on the influence of robot personality." International Conference on Intelligent Robots and Systems.
3. Chen, G., et al. (2018). "A survey of social robot control through natural language." ACM Transactions on Interactive Intelligent Systems.
4. Kory Westlund, J. M., et al. (2017). "Improving theory of mind with stories in a field trial with 10-month-old children." Proceedings of the 2017 CHI Conference on Human Factors in Computing Systems.
5. Breazeal, C. (2003). "Emotion and sociable humanoid robots." International Journal of Human-Computer Studies.
6. Marge, M., et al. (2019). "Real-time incremental speech recognition and understanding for human-robot interaction." IEEE International Conference on Robotics and Automation.
7. Glass, J. (2009). "Applications of automatic speech recognition in human-robot interaction." IEEE Signal Processing Magazine.