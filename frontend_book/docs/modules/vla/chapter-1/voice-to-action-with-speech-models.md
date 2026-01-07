---
sidebar_position: 1
---

# Chapter 1: Voice-to-Action with Speech Models

## Understanding Speech-to-Text Systems and Their Integration

Speech-to-text (STT) systems form the foundational component of voice-controlled robotic systems. These systems convert spoken language into written text that can be processed by robotic control systems. For humanoid robots, speech-to-text integration enables natural human-robot interaction through voice commands.

### Key Components of Speech-to-Text Systems

Modern STT systems consist of several key components:

- **Acoustic Model**: Maps audio signals to phonetic units
- **Language Model**: Predicts the most likely sequence of words given the acoustic model
- **Decoder**: Combines acoustic and language models to produce text output
- **Audio Processing Pipeline**: Handles preprocessing of audio signals for optimal recognition

### Popular Speech Recognition APIs

Several APIs and frameworks are commonly used for speech recognition in robotics:

- **Google Speech-to-Text API**: Highly accurate cloud-based service
- **Microsoft Azure Speech Service**: Enterprise-grade speech recognition
- **Mozilla DeepSpeech**: Open-source speech recognition engine
- **Kaldi ASR**: Flexible toolkit for speech recognition research
- **Vosk**: Lightweight offline speech recognition

### Audio Preprocessing for Robotic Applications

For robotic applications, audio preprocessing is critical for accurate recognition:

```python
import numpy as np
import scipy.signal as signal
from scipy.io import wavfile

def preprocess_audio(audio_signal, sample_rate=16000):
    """
    Preprocess audio for robotic speech recognition
    """
    # Normalize audio
    audio_signal = audio_signal.astype(np.float32)
    audio_signal /= np.max(np.abs(audio_signal))
    
    # Apply noise reduction
    # Using spectral subtraction method
    # (Simplified implementation for demonstration)
    
    # Bandpass filtering to focus on human voice frequencies
    nyquist = sample_rate / 2
    low_freq = 80 / nyquist  # Low end of human voice
    high_freq = 3400 / nyquist  # High end of human voice
    
    b, a = signal.butter(4, [low_freq, high_freq], btype='band')
    filtered_audio = signal.filtfilt(b, a, audio_signal)
    
    # Return processed audio
    return filtered_audio

def enhance_speech_features(audio_signal):
    """
    Enhance speech features for better recognition
    """
    # Calculate Mel-frequency cepstral coefficients (MFCCs)
    # This is a simplified approach - full implementation would be more complex
    window_size = int(0.025 * 16000)  # 25ms window
    hop_size = int(0.01 * 16000)     # 10ms hop
    
    # Window the signal
    frames = []
    for i in range(0, len(audio_signal) - window_size, hop_size):
        frame = audio_signal[i:i + window_size]
        # Apply Hamming window
        frame = frame * np.hamming(window_size)
        frames.append(frame)
    
    return np.array(frames)
```

## Processing Spoken Commands for Robotic Applications

Processing spoken commands for robotics requires special considerations beyond standard speech recognition:

### Command Structure and Grammar

Robotic commands typically follow structured grammars:

```
[Action] [Object] [Location/Parameter]
Examples:
- "Go to the kitchen"
- "Pick up the red cup"
- "Turn off the lights"
- "Bring me the book from the table"
```

### Intent Recognition

Once speech is converted to text, intent recognition determines the robot's action:

```python
import re
from enum import Enum

class RobotIntent(Enum):
    NAVIGATE = "navigate"
    GRASP_OBJECT = "grasp"
    MANIPULATE = "manipulate"
    SPEAK = "speak"
    FOLLOW = "follow"
    STOP = "stop"

class CommandRecognizer:
    def __init__(self):
        # Define patterns for different intents
        self.patterns = {
            RobotIntent.NAVIGATE: [
                r"go to (?P<location>\w+)",
                r"move to (?P<location>\w+)",
                r"navigate to (?P<location>\w+)",
                r"reach (?P<location>\w+)"
            ],
            RobotIntent.GRASP_OBJECT: [
                r"pick up (?P<object>\w+)",
                r"take (?P<object>\w+)",
                r"grab (?P<object>\w+)",
                r"lift (?P<object>\w+)"
            ],
            RobotIntent.MANIPULATE: [
                r"turn (?P<direction>\w+) (?P<object>\w+)",
                r"open (?P<object>\w+)",
                r"close (?P<object>\w+)",
                r"switch (?P<action>\w+) (?P<object>\w+)"
            ],
            RobotIntent.SPEAK: [
                r"say (?P<message>.+)",
                r"speak (?P<message>.+)"
            ],
            RobotIntent.STOP: [
                r"stop",
                r"halt",
                r"pause"
            ]
        }
    
    def recognize_intent(self, text):
        """
        Recognize intent from text command
        """
        text_lower = text.lower().strip()
        
        for intent, patterns in self.patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    return intent, match.groupdict()
        
        # If no specific pattern matches, return a general command
        return None, {"raw_command": text_lower}

# Example usage
recognizer = CommandRecognizer()
intent, params = recognizer.recognize_intent("Go to the kitchen")
print(f"Intent: {intent}, Params: {params}")
```

## Converting Speech to Structured Robot Instructions

Converting speech to structured instructions involves mapping natural language to robot commands:

### Natural Language to Action Mapping

```python
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class RobotInstruction:
    action: str
    parameters: Dict[str, Any]
    confidence: float
    priority: int = 1

class SpeechToInstructionConverter:
    def __init__(self):
        self.command_recognizer = CommandRecognizer()
        self.location_mapper = LocationMapper()
        self.object_detector = ObjectDetector()
    
    def convert_speech_to_instruction(self, speech_text, confidence_score):
        """
        Convert speech text to structured robot instruction
        """
        intent, params = self.command_recognizer.recognize_intent(speech_text)
        
        if intent == RobotIntent.NAVIGATE:
            # Map location name to coordinates
            location_coords = self.location_mapper.get_coordinates(params.get('location', ''))
            return RobotInstruction(
                action='navigate',
                parameters={'target_location': location_coords},
                confidence=confidence_score
            )
        
        elif intent == RobotIntent.GRASP_OBJECT:
            # Identify object and its location
            object_info = self.object_detector.find_object(params.get('object', ''))
            return RobotInstruction(
                action='grasp_object',
                parameters={
                    'object_name': params.get('object'),
                    'object_pose': object_info.pose if object_info else None
                },
                confidence=confidence_score
            )
        
        elif intent == RobotIntent.MANIPULATE:
            return RobotInstruction(
                action='manipulate_object',
                parameters={
                    'object_name': params.get('object'),
                    'manipulation_type': params.get('action', 'toggle')
                },
                confidence=confidence_score
            )
        
        # Handle other intents similarly
        return None

class LocationMapper:
    def __init__(self):
        # Predefined locations in the environment
        self.locations = {
            'kitchen': {'x': 2.0, 'y': 1.0, 'theta': 0.0},
            'living_room': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'bedroom': {'x': -1.0, 'y': 2.0, 'theta': 1.57},
            'office': {'x': 1.5, 'y': -1.0, 'theta': -1.57}
        }
    
    def get_coordinates(self, location_name):
        """
        Get coordinates for a named location
        """
        return self.locations.get(location_name.lower())

class ObjectDetector:
    def find_object(self, object_name):
        """
        Simulate finding an object in the environment
        """
        # In a real implementation, this would interface with perception systems
        class MockObject:
            def __init__(self, name):
                self.name = name
                self.pose = {'x': 0.5, 'y': 0.3, 'z': 0.8, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}
        
        return MockObject(object_name)
```

## Handling Speech Recognition Errors and Uncertainties

Speech recognition in real-world environments faces several challenges:

### Confidence Scoring and Validation

```python
class SpeechValidator:
    def __init__(self):
        self.confidence_threshold = 0.7
        self.ambiguity_threshold = 0.3
    
    def validate_recognition(self, text, confidence, alternatives=None):
        """
        Validate speech recognition results
        """
        result = {
            'is_valid': confidence > self.confidence_threshold,
            'text': text,
            'confidence': confidence,
            'needs_confirmation': False,
            'alternatives': alternatives or []
        }
        
        # Check for ambiguous results
        if alternatives and len(alternatives) > 1:
            # Calculate ambiguity score based on confidence differences
            confidences = [alt['confidence'] for alt in alternatives]
            if len(confidences) > 1:
                top_two_diff = confidences[0] - confidences[1]
                if top_two_diff < self.ambiguity_threshold:
                    result['needs_confirmation'] = True
        
        return result

def handle_recognition_errors(recognized_text, confidence_score, alternatives=None):
    """
    Handle speech recognition errors and uncertainties
    """
    validator = SpeechValidator()
    validation_result = validator.validate_recognition(
        recognized_text, 
        confidence_score, 
        alternatives
    )
    
    if not validation_result['is_valid']:
        # Low confidence - request repetition
        return {
            'action': 'request_repitition',
            'reason': 'low_confidence',
            'original_text': recognized_text
        }
    
    if validation_result['needs_confirmation']:
        # Ambiguous result - ask for confirmation
        return {
            'action': 'request_confirmation',
            'options': [alt['text'] for alt in validation_result['alternatives']],
            'original_text': recognized_text
        }
    
    # Valid recognition - proceed with command processing
    return {
        'action': 'process_command',
        'command': validation_result['text'],
        'confidence': validation_result['confidence']
    }
```

## Voice Command Validation and Safety Considerations

### Command Validation Pipeline

```python
class VoiceCommandValidator:
    def __init__(self):
        self.safe_actions = ['navigate', 'speak', 'wave', 'greet']
        self.dangerous_keywords = ['kill', 'harm', 'destroy', 'attack']
        self.emergency_commands = ['stop', 'emergency_stop', 'halt']
    
    def validate_command(self, instruction: RobotInstruction):
        """
        Validate robot instruction for safety and appropriateness
        """
        # Check for dangerous keywords
        if hasattr(instruction, 'parameters') and 'raw_command' in instruction.parameters:
            raw_text = instruction.parameters['raw_command'].lower()
            for keyword in self.dangerous_keywords:
                if keyword in raw_text:
                    return {
                        'valid': False,
                        'reason': f'Dangerous keyword detected: {keyword}',
                        'severity': 'critical'
                    }
        
        # Check if action is in safe list
        if instruction.action not in self.safe_actions:
            return {
                'valid': False,
                'reason': f'Action {instruction.action} not in safe actions list',
                'severity': 'warning'
            }
        
        # Validate parameters
        validation_errors = self.validate_parameters(instruction)
        if validation_errors:
            return {
                'valid': False,
                'reason': f'Parameter validation failed: {validation_errors}',
                'severity': 'error'
            }
        
        # All checks passed
        return {
            'valid': True,
            'reason': 'All validations passed',
            'severity': 'info'
        }
    
    def validate_parameters(self, instruction: RobotInstruction):
        """
        Validate specific parameters for the instruction
        """
        errors = []
        
        if instruction.action == 'navigate':
            # Validate navigation parameters
            if 'target_location' not in instruction.parameters:
                errors.append('Missing target_location parameter')
            else:
                loc = instruction.parameters['target_location']
                if not isinstance(loc, dict) or 'x' not in loc or 'y' not in loc:
                    errors.append('Invalid location format')
        
        elif instruction.action == 'grasp_object':
            # Validate grasp parameters
            if 'object_name' not in instruction.parameters:
                errors.append('Missing object_name parameter')
        
        return errors
```

## Integration with ROS 2 Command Systems

### ROS 2 Speech Interface

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import AudioData
from vla_interfaces.srv import ProcessVoiceCommand  # Custom service
from vla_interfaces.msg import RobotInstruction as RobotInstructionMsg

class SpeechToActionNode(Node):
    def __init__(self):
        super().__init__('speech_to_action_node')
        
        # Publishers
        self.instruction_publisher = self.create_publisher(
            RobotInstructionMsg, 
            'robot_instructions', 
            10
        )
        
        # Subscribers
        self.audio_subscriber = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )
        
        self.text_subscriber = self.create_subscription(
            String,
            'recognized_text',
            self.text_callback,
            10
        )
        
        # Services
        self.voice_command_service = self.create_service(
            ProcessVoiceCommand,
            'process_voice_command',
            self.process_voice_command_callback
        )
        
        # Initialize components
        self.speech_converter = SpeechToInstructionConverter()
        self.validator = VoiceCommandValidator()
        self.confidence_threshold = 0.7
    
    def audio_callback(self, msg):
        """
        Process incoming audio data
        """
        # In a real implementation, this would interface with STT system
        # For this example, we'll assume text comes from another node
        pass
    
    def text_callback(self, msg):
        """
        Process recognized text from STT system
        """
        # Convert text to instruction
        instruction = self.speech_converter.convert_speech_to_instruction(
            msg.data, 
            self.confidence_threshold
        )
        
        if instruction:
            # Validate instruction
            validation_result = self.validator.validate_command(instruction)
            
            if validation_result['valid']:
                # Publish instruction
                self.publish_instruction(instruction)
            else:
                self.get_logger().warn(
                    f"Command validation failed: {validation_result['reason']}"
                )
    
    def process_voice_command_callback(self, request, response):
        """
        Service callback for processing voice commands
        """
        instruction = self.speech_converter.convert_speech_to_instruction(
            request.text, 
            request.confidence
        )
        
        if instruction:
            validation_result = self.validator.validate_command(instruction)
            
            if validation_result['valid']:
                self.publish_instruction(instruction)
                response.success = True
                response.message = "Command processed successfully"
            else:
                response.success = False
                response.message = f"Validation failed: {validation_result['reason']}"
        else:
            response.success = False
            response.message = "Could not convert speech to instruction"
        
        return response
    
    def publish_instruction(self, instruction):
        """
        Publish robot instruction to command system
        """
        msg = RobotInstructionMsg()
        msg.action = instruction.action
        msg.confidence = instruction.confidence
        msg.priority = instruction.priority
        
        # Convert parameters dictionary to message format
        for key, value in instruction.parameters.items():
            param = String()
            param.data = f"{key}:{str(value)}"
            msg.parameters.append(param)
        
        self.instruction_publisher.publish(msg)
        self.get_logger().info(f"Published instruction: {instruction.action}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToActionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Voice Interface Design for Human-Robot Interaction

### Design Principles for Voice Interfaces

Effective voice interfaces for humanoid robots should follow these principles:

1. **Natural Language Understanding**: Allow users to speak naturally rather than using rigid command structures
2. **Feedback Mechanisms**: Provide clear feedback when commands are received and executed
3. **Error Recovery**: Gracefully handle misrecognitions with confirmation mechanisms
4. **Context Awareness**: Understand commands in context of the current situation
5. **Historical Learning**: Use past interactions to improve future responses
6. **Robustness**: Function reliably in noisy environments

### Interaction History and Context Management

To create more natural and effective voice interactions, humanoid robots should maintain a history of past interactions and use this information to improve future responses:

```python
import datetime
from enum import Enum
from typing import List, Dict, Any, Optional
from dataclasses import dataclass

class InteractionType(Enum):
    VOICE_COMMAND = "voice_command"
    TEXT_COMMAND = "text_command"
    SYSTEM_RESPONSE = "system_response"
    ERROR_EVENT = "error_event"

@dataclass
class InteractionRecord:
    """Represents a single interaction with timestamp and metadata"""
    id: str
    timestamp: datetime.datetime
    interaction_type: InteractionType
    content: str
    context: Dict[str, Any]
    result: Optional[str] = None
    duration: Optional[float] = None
    confidence: Optional[float] = None

class HistoryManager:
    """Manages interaction history for the voice interface"""

    def __init__(self, max_history_length: int = 100):
        self.history: List[InteractionRecord] = []
        self.max_history_length = max_history_length

    def add_interaction(self, record: InteractionRecord):
        """Add a new interaction to the history"""
        self.history.append(record)

        # Maintain history size limit
        if len(self.history) > self.max_history_length:
            self.history = self.history[-self.max_history_length:]

    def get_recent_interactions(self, count: int = 10) -> List[InteractionRecord]:
        """Get the most recent interactions"""
        return self.history[-count:] if len(self.history) >= count else self.history[:]

    def get_context_for_next_interaction(self) -> Dict[str, Any]:
        """Generate context for the next interaction based on history"""
        recent_interactions = self.get_recent_interactions(5)
        context = {
            'total_interactions': len(self.history),
            'recent_successful_commands': [],
            'user_preferences': {},
            'common_phrases': {}
        }

        # Extract successful commands
        for interaction in recent_interactions:
            if (interaction.interaction_type == InteractionType.VOICE_COMMAND and
                interaction.result == 'success'):
                context['recent_successful_commands'].append(interaction.content)

        # Count common phrases
        for interaction in recent_interactions:
            if interaction.interaction_type == InteractionType.VOICE_COMMAND:
                cmd = interaction.content.lower()
                context['common_phrases'][cmd] = context['common_phrases'].get(cmd, 0) + 1

        return context

class VoiceInteractionManager:
    def __init__(self):
        self.context = {}  # Store conversation context
        self.conversation_state = 'idle'
        self.last_command_time = None
        self.history_manager = HistoryManager()

    def process_voice_interaction(self, recognized_text, confidence):
        """
        Process voice interaction with context awareness and history tracking
        """
        # Add current interaction to history
        interaction_record = InteractionRecord(
            id=f"interaction_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')}",
            timestamp=datetime.datetime.now(),
            interaction_type=InteractionType.VOICE_COMMAND,
            content=recognized_text,
            context={'confidence': confidence, 'state': self.conversation_state},
            confidence=confidence
        )
        self.history_manager.add_interaction(interaction_record)

        # Update context based on history
        self.context.update(self.history_manager.get_context_for_next_interaction())

        # Update context based on current state
        if self.conversation_state == 'awaiting_confirmation':
            return self.handle_confirmation(recognized_text)

        # Validate recognition
        validation = handle_recognition_errors(recognized_text, confidence)

        if validation['action'] == 'request_repitition':
            return self.request_repitition()

        elif validation['action'] == 'request_confirmation':
            return self.request_confirmation(validation['options'])

        elif validation['action'] == 'process_command':
            result = self.execute_command(validation['command'])
            # Update history with execution result
            interaction_record.result = 'success' if 'Okay, I will' in result.get('response', '') else 'failure'
            return result

    def handle_confirmation(self, user_response):
        """
        Handle user response to confirmation request
        """
        response_lower = user_response.lower()

        if 'yes' in response_lower or 'correct' in response_lower:
            # User confirmed the command
            command_to_execute = self.context.get('pending_command')
            if command_to_execute:
                return self.execute_command(command_to_execute)

        elif 'no' in response_lower or 'wrong' in response_lower:
            # User rejected the interpretation
            return self.request_repitition()

        else:
            # Ambiguous response - ask again
            return self.request_clear_confirmation()

    def request_repitition(self):
        """
        Request user to repeat the command
        """
        self.conversation_state = 'awaiting_command'
        return {
            'response': 'I didn\'t catch that. Could you please repeat your command?',
            'action': 'listen_for_command'
        }

    def request_confirmation(self, options):
        """
        Request user to confirm the intended command
        """
        self.context['pending_command'] = options[0]  # Assume first option is most likely
        self.conversation_state = 'awaiting_confirmation'

        return {
            'response': f'Did you mean "{options[0]}"? Please say yes or no.',
            'action': 'listen_for_confirmation'
        }

    def execute_command(self, command):
        """
        Execute the validated command
        """
        # Convert to instruction and publish
        instruction = self.speech_converter.convert_speech_to_instruction(
            command,
            confidence=0.9  # High confidence since user confirmed
        )

        if instruction:
            validation_result = self.validator.validate_command(instruction)
            if validation_result['valid']:
                # Publish instruction and provide feedback
                self.publish_instruction(instruction)
                return {
                    'response': f'Okay, I will {command}.',
                    'action': 'execute_instruction'
                }

        return {
            'response': 'I encountered an issue processing that command.',
            'action': 'idle'
        }
```

The voice-to-action pipeline forms a critical component of the Vision-Language-Action system, enabling natural human-robot interaction through spoken commands. Proper integration of speech recognition, intent parsing, validation, and safety checks ensures reliable and safe operation of humanoid robots in human environments. The addition of history tracking and context management allows the system to learn from past interactions and provide more personalized and effective responses over time.