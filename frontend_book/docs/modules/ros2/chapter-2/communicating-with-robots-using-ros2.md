---
sidebar_position: 2
---

# Chapter 2: Communicating with Robots Using ROS 2

## ROS 2 Nodes Lifecycle

In ROS 2, nodes have a well-defined lifecycle that governs their behavior from creation to destruction. Understanding this lifecycle is crucial for developing robust robotic applications.

### Node States

A ROS 2 node transitions through several states:

- **Unconfigured**: Initial state after node creation
- **Inactive**: After successful configuration
- **Active**: After successful activation, ready to execute
- **Finalized**: After successful cleanup, ready for destruction

### Lifecycle Node Implementation

Lifecycle nodes provide better control over the startup and shutdown processes:

- **on_configure()**: Called when transitioning from unconfigured to inactive
- **on_activate()**: Called when transitioning from inactive to active
- **on_deactivate()**: Called when transitioning from active to inactive
- **on_cleanup()**: Called when transitioning from inactive to unconfigured
- **on_shutdown()**: Called when transitioning from any state to finalized

This structured approach enables better resource management and error handling in complex robotic systems.

## Topics vs Services vs Actions (Conceptual Level)

Understanding the differences between these communication patterns is essential for effective ROS 2 design:

### Topics (Publish-Subscribe)

**Use Case**: Continuous data streams, sensor data, state information
- Asynchronous communication
- Multiple publishers and subscribers possible
- Decoupled in time and space
- Best for data that changes continuously

**Example**: Camera images, laser scan data, robot pose

### Services (Request-Response)

**Use Case**: Discrete operations, computations, state queries
- Synchronous communication
- One request, one response pattern
- Blocking until response received
- Best for operations with clear start and end

**Example**: Map saving, trajectory planning, parameter queries

### Actions (Goal-Feedback-Result)

**Use Case**: Long-running tasks with progress feedback
- Asynchronous with feedback capability
- Supports goal preemption and cancelation
- Multiple states: pending, active, succeeded, aborted, canceled
- Best for tasks that take time and need monitoring

**Example**: Navigation to goal, object manipulation, trajectory execution

## Python Agents Interacting with Robots via rclpy

The `rclpy` library provides Python bindings for ROS 2, enabling Python-based AI agents to interact with robots seamlessly.

### Setting Up a ROS 2 Python Node

```python
import rclpy
from rclpy.node import Node

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        # Initialize publishers, subscribers, services, etc.
```

### Creating Publishers and Subscribers

```python
# Creating a publisher
publisher = self.create_publisher(String, 'topic_name', 10)

# Creating a subscriber
subscriber = self.create_subscription(
    String,
    'topic_name',
    self.callback_function,
    10
)

def callback_function(self, msg):
    # Process incoming message
    self.get_logger().info(f'Received: {msg.data}')
```

### Working with Services

```python
# Creating a service client
client = self.create_client(ServiceType, 'service_name')

# Creating a service server
service = self.create_service(ServiceType, 'service_name', self.service_callback)

def service_callback(self, request, response):
    # Process service request and return response
    return response
```

## Mapping AI Decisions to Actuator Commands

The bridge between AI decision-making and physical robot control involves translating high-level AI outputs into low-level actuator commands.

### Conceptual Mapping Process

1. **AI Decision**: High-level action selection (e.g., "move to location A")
2. **Path Planning**: Convert to a sequence of waypoints
3. **Trajectory Generation**: Create time-parameterized motion commands
4. **Low-level Control**: Convert to actuator commands (motor speeds, joint angles)
5. **Execution**: Send commands to robot hardware

### Implementation Pattern

```python
class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Publishers for different robot interfaces
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # Subscribers for sensor data
        self.sensor_subscriber = self.create_subscription(
            LaserScan, '/scan', self.sensor_callback, 10
        )

        # Timer for AI decision loop
        self.timer = self.create_timer(0.1, self.ai_decision_loop)

    def ai_decision_loop(self):
        # Process AI decision logic
        ai_decision = self.process_ai_model()

        # Map decision to robot commands
        robot_cmd = self.map_ai_to_robot(ai_decision)

        # Publish command to robot
        self.cmd_vel_publisher.publish(robot_cmd)

    def map_ai_to_robot(self, ai_decision):
        # Convert AI decision to appropriate robot command
        cmd = Twist()
        if ai_decision == "move_forward":
            cmd.linear.x = 0.5
        elif ai_decision == "turn_left":
            cmd.angular.z = 0.5
        return cmd
```

## Practical Examples of Communication Patterns

### Example 1: Object Detection and Navigation

```python
class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publish detected objects
        self.obj_pub = self.create_publisher(ObjectList, '/detected_objects', 10)

    def image_callback(self, msg):
        # Process image with AI model
        objects = self.ai_object_detection(msg)

        # Publish detected objects
        obj_msg = ObjectList()
        obj_msg.objects = objects
        self.obj_pub.publish(obj_msg)
```

### Example 2: Navigation Action Server

```python
class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        # Create action server
        self.nav_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_navigate_goal
        )

    def execute_navigate_goal(self, goal_handle):
        # Execute navigation goal with feedback
        feedback_msg = NavigateToPose.Feedback()

        while not self.reached_goal():
            # Publish feedback
            feedback_msg.current_pose = self.get_current_pose()
            goal_handle.publish_feedback(feedback_msg)

            # Check for cancelation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

        # Goal succeeded
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result
```

This communication architecture enables AI agents to interact with robots at multiple levels of abstraction, from low-level sensor data to high-level task planning, creating a comprehensive nervous system for intelligent robotic behavior.