---
sidebar_position: 3
title: 'Chapter 1.2 — ROS 2 with Python (rclpy)'
---

# Chapter 1.2: ROS 2 with Python (rclpy)

## Introduction

Python is one of the most popular programming languages in robotics and AI development, and ROS 2 provides excellent Python support through the `rclpy` client library. This chapter will guide you through the fundamentals of using Python with ROS 2, from basic node creation to advanced communication patterns.

## Learning Objectives

By the end of this chapter, you should be able to:
1. Create and run basic ROS 2 nodes using Python
2. Implement publishers and subscribers in Python
3. Create and use services and clients in Python
4. Work with actions using Python
5. Understand the rclpy library structure and best practices

## Getting Started with rclpy

The `rclpy` library provides a Python interface to the ROS 2 client library. It wraps the C-based ROS client library (`rcl`) to provide a Pythonic API for ROS 2 development.

### Basic Node Structure

A minimal ROS 2 node in Python follows this structure:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating Your First Node

Let's create a simple node that logs messages:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers and Subscribers

### Publishers

Publishers send messages to topics. Here's how to create a publisher:

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello from publisher'
        self.publisher.publish(msg)
```

### Subscribers

Subscribers receive messages from topics:

```python
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services and Clients

### Services

Services provide request-response communication:

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Clients

Clients call services:

```python
from example_interfaces.srv import AddTwoInts

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions handle long-running tasks with feedback:

```python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info('Publishing feedback: {0}'.format(
                feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))
        return result
```

## Working with Parameters

ROS 2 nodes can have configurable parameters:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('integer_param', 42)
        self.declare_parameter('float_param', 3.14)

        # Get parameter values
        param_value = self.get_parameter('param_name').value
        self.get_logger().info(f'Parameter value: {param_value}')
```

## Quality of Service (QoS) in Python

QoS settings control how messages are delivered:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Create a QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Use in publisher/subscriber
publisher = self.create_publisher(String, 'topic', qos_profile)
```

## Best Practices for rclpy

1. **Always handle exceptions**: Wrap ROS operations in try-catch blocks
2. **Use proper node lifecycle**: Initialize, spin, and cleanup properly
3. **Manage resources**: Destroy nodes and clean up resources when done
4. **Log appropriately**: Use the built-in logger for debugging and monitoring
5. **Follow naming conventions**: Use consistent naming for topics, services, and parameters

## Exercises

1. **Basic Publisher/Subscriber**: Create a publisher node that sends temperature readings and a subscriber node that logs these readings. Add a parameter to control the publishing rate.

2. **Service Implementation**: Implement a service that converts temperatures between Celsius and Fahrenheit. Create a client that uses this service.

3. **Action Practice**: Create an action that simulates a robot moving to a specified position. The action should provide feedback on the progress and return the final position.

## References

1. ROS 2 Documentation. (2023). "rclpy Developer Guide". https://docs.ros.org/en/rolling/How-To-Guides/Using-RCLPY-Wrappers.html
2. ROS 2 Documentation. (2023). "Writing a Simple Publisher and Subscriber (Python)". https://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
3. ROS 2 Documentation. (2023). "Writing a Simple Service and Client (Python)". https://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Service-And-Client.html

## Advanced rclpy Concepts

Beyond the basic patterns, rclpy offers several advanced features for complex robotics applications:

### Callback Groups

Callback groups allow you to control how callbacks are executed:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# Mutually exclusive: only one callback runs at a time
exclusive_group = MutuallyExclusiveCallbackGroup()

# Reentrant: multiple callbacks can run simultaneously
reentrant_group = ReentrantCallbackGroup()

# Using in subscriptions
sub = self.create_subscription(
    String,
    'topic',
    self.callback,
    10,
    callback_group=exclusive_group)
```

### Timers

Timers are essential for periodic tasks:

```python
def timer_callback(self):
    self.get_logger().info('Timer activated')

# Create a timer with 0.5 second period
timer = self.create_timer(0.5, self.timer_callback)
```

### Multi-threading and Executors

ROS 2 provides executors to handle multiple nodes and callbacks:

```python
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

# Single threaded executor (default)
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()

# Multi-threaded executor for concurrent processing
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
```

### Lifecycle Nodes

For complex systems requiring state management:

```python
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleNodeExample(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating')
        return TransitionCallbackReturn.SUCCESS
```

## Message and Service Definitions

Creating custom messages and services:

### Custom Message (msg/MyMessage.msg)
```
string name
int32 id
float64[] position
bool active
```

### Custom Service (srv/MyService.srv)
```
string input
---
bool success
string message
```

### Using Custom Messages
```python
from my_package_msgs.msg import MyMessage
from my_package_msgs.srv import MyService
```

## Launch Files in Python

Python launch files provide more flexibility:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[
                {'param1': 'value1'},
                {'param2': 42}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        )
    ])
```

## Parameter Handling

Advanced parameter features:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with descriptions
        self.declare_parameter(
            'robot_name',
            'default_robot',
            descriptor=ParameterDescriptor(
                description='Name of the robot',
                type=ParameterType.PARAMETER_STRING
            )
        )

        # Parameter callbacks
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, parameters):
        for param in parameters:
            if param.name == 'robot_name' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f'Robot name changed to: {param.value}')
        return SetParametersResult(successful=True)
```

## Testing with rclpy

Writing tests for ROS 2 nodes:

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

class TestMyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_publisher_subscriber(self):
        # Create publisher and subscriber nodes
        publisher_node = PublisherNode()
        subscriber_node = SubscriberNode()

        executor = SingleThreadedExecutor()
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)

        # Publish and verify
        publisher_node.publish_message()
        executor.spin_once(timeout_sec=1.0)

        # Add assertions here
```

## Performance Considerations

### Memory Management
- Use object pooling for high-frequency messages
- Pre-allocate message objects when possible
- Consider using `loaned messages` for zero-copy scenarios

### CPU Optimization
- Use appropriate QoS settings for your application
- Implement efficient callback functions
- Consider using composition for performance-critical applications

### Network Efficiency
- Use appropriate message types and sizes
- Implement data compression for large messages
- Optimize update frequencies based on requirements

## Error Handling and Debugging

### Common Error Patterns
```python
def robust_callback(self, msg):
    try:
        # Process message
        result = self.process_data(msg.data)
        self.publish_result(result)
    except ValueError as e:
        self.get_logger().error(f'Invalid data: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error: {e}')
```

### Logging Best Practices
```python
# Use different log levels appropriately
self.get_logger().debug('Detailed debug information')
self.get_logger().info('Normal operational information')
self.get_logger().warn('Warning about potential issues')
self.get_logger().error('Error occurred but node can continue')
self.get_logger().fatal('Critical error requiring node shutdown')
```

## Integration with External Libraries

### NumPy Integration
```python
import numpy as np
from std_msgs.msg import Float64MultiArray

def numpy_to_ros(self, data):
    msg = Float64MultiArray()
    msg.data = data.flatten().tolist()
    return msg
```

### OpenCV Integration
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageNode(Node):
    def __init__(self):
        super().__init__('image_node')
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Process with OpenCV
        processed_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
```

## Best Practices for Production

### Code Structure
- Separate ROS-specific code from business logic
- Use dependency injection for testing
- Implement proper error handling
- Follow PEP 8 guidelines

### Configuration Management
- Use parameters for runtime configuration
- Support different environments (simulation vs real robot)
- Implement configuration validation

### Resource Management
- Properly clean up resources in `destroy_node()`
- Monitor memory and CPU usage
- Implement graceful degradation under load

## Common Design Patterns

### Publisher-Subscriber Pattern
```python
class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(
            PointCloud2, 'processed_scan', 10)
```

### Client-Service Pattern
```python
class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        self.client = self.create_client(
            GetPlan, 'global_planner/get_plan')
```

### Action Client Pattern
```python
class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
```

## Summary

Python with rclpy provides a powerful and accessible way to develop ROS 2 applications. The library offers a clean, Pythonic interface to ROS 2's core functionality, making it ideal for rapid prototyping, educational purposes, and production systems. Understanding how to properly structure nodes, handle communication patterns, and follow best practices is essential for effective ROS 2 development in Python. With advanced features like callback groups, lifecycle nodes, and comprehensive testing capabilities, rclpy enables developers to build robust and efficient robotic applications that can handle the complexities of real-world robotics systems.