---
sidebar_position: 2
title: 'Chapter 1.1 — ROS 2 Architecture'
---

# Chapter 1.1: ROS 2 Architecture

## Introduction

The Robot Operating System 2 (ROS 2) represents a significant evolution from its predecessor, designed specifically to address the needs of production robotics applications. Unlike ROS 1, which was primarily focused on research environments, ROS 2 incorporates industrial-grade features such as improved security, real-time support, and multi-robot systems.

## Learning Objectives

By the end of this chapter, you should be able to:
1. Explain the core architecture of ROS 2
2. Identify the key differences between ROS 1 and ROS 2
3. Understand the role of DDS in ROS 2 communication
4. Describe the main components of the ROS 2 ecosystem

## What is ROS 2?

ROS 2 is not just an update to the original ROS; it's a complete redesign addressing the limitations of the original system. The primary goal was to create a framework suitable for commercial and industrial applications, with enhanced security, reliability, and scalability.

### Key Features of ROS 2

1. **Real-time support**: Enables time-critical applications with deterministic behavior
2. **Improved security**: Built-in security features for safe multi-robot deployments
3. **Multi-robot systems**: Better support for coordinating multiple robots
4. **Cross-platform compatibility**: Runs on various operating systems including Linux, Windows, and macOS
5. **Quality of Service (QoS) settings**: Configurable reliability and performance options

## DDS: The Communication Backbone

The most significant architectural change in ROS 2 is the adoption of Data Distribution Service (DDS) as the underlying communication middleware. DDS provides:

- **Publisher-Subscriber communication**: The core communication pattern in ROS 2
- **Service-Client communication**: For request-response interactions
- **Action communication**: For long-running tasks with feedback
- **Built-in Quality of Service policies**: For controlling reliability, durability, and other communication aspects

### Why DDS?

DDS was chosen because it provides:
- **Decentralized architecture**: No single point of failure
- **Language independence**: Supports multiple programming languages
- **Platform independence**: Works across different operating systems
- **Real-time capabilities**: Suitable for time-critical applications
- **Standardized specification**: Ensures interoperability

## Core Architecture Components

### Nodes

In ROS 2, a node is a fundamental component that represents a single process. Each node:

- Contains publishers, subscribers, services, or actions
- Communicates with other nodes through the ROS graph
- Has a unique name within the ROS domain
- Can be written in any supported language (C++, Python, etc.)

### Topics

Topics are named buses over which nodes exchange messages. Key characteristics:

- Unidirectional communication (publisher → subscriber)
- Multiple publishers and subscribers can use the same topic
- Implemented using DDS publish-subscribe pattern
- Support Quality of Service (QoS) configurations

### Services

Services provide synchronous request-response communication:

- One client sends a request to one service
- The service processes the request and returns a response
- Blocking communication pattern
- Useful for operations that must complete before proceeding

### Actions

Actions handle long-running tasks with feedback:

- Asynchronous communication pattern
- Supports goal, feedback, and result messages
- Allows for task preemption
- Ideal for navigation, manipulation, and other extended operations

## ROS 2 Ecosystem

The ROS 2 ecosystem includes several key components:

### RMW (ROS Middleware) Layer

The ROS Middleware layer abstracts the underlying DDS implementation, allowing:

- Switching between different DDS vendors (Fast DDS, Cyclone DDS, RTI Connext)
- Maintaining API consistency across different middleware implementations
- Selecting the most appropriate DDS implementation for specific use cases

### rcl and rclcpp/rclpy

- **rcl**: Client library implementation in C
- **rclcpp**: C++ wrapper around rcl
- **rclpy**: Python wrapper around rcl

These libraries provide the common interface that all client libraries use to interact with the ROS 2 system.

## Domain IDs and Isolation

ROS 2 uses domain IDs to create isolated communication spaces:

- Nodes in different domains cannot communicate
- Allows multiple ROS 2 systems on the same network
- Provides security through isolation
- Configurable via environment variables or launch files

## Exercises

1. **Conceptual Understanding**: Explain the difference between topics, services, and actions in ROS 2. When would you use each communication pattern?

2. **Practical Exercise**: Set up a basic ROS 2 environment on your system. Create two nodes that communicate over a custom topic.

## References

1. ROS 2 Documentation. (2023). "ROS 2 Design". https://design.ros2.org/
2. ROS 2 Documentation. (2023). "DDS and ROS 2". https://docs.ros.org/en/rolling/Concepts/About-Data-Distribution-Service.html
3. Perez, A., et al. (2020). "ROS 2: New challenges and opportunities for robotics development". Communications of the ACM, 63(5), 66-74.

## Communication Patterns in Depth

Understanding the communication patterns in ROS 2 is crucial for effective system design. Each pattern serves different purposes and has specific use cases:

### Publisher-Subscriber (Topics)
- **Asynchronous**: Publishers and subscribers don't need to be active simultaneously
- **One-to-many**: One publisher can send to multiple subscribers
- **Data flow**: Unidirectional from publisher to subscriber
- **Use cases**: Sensor data, robot state, continuous monitoring

### Service-Client
- **Synchronous**: Client waits for response from service
- **One-to-one**: One client communicates with one service at a time
- **Request-response**: Client sends request, service returns response
- **Use cases**: Database queries, transformations, blocking operations

### Actions
- **Asynchronous with feedback**: Long-running operations with status updates
- **Goal-based**: Client sends goal, receives feedback and result
- **Cancel support**: Operations can be canceled during execution
- **Use cases**: Navigation, manipulation, calibration

## Quality of Service (QoS) Policies

ROS 2 provides Quality of Service policies to control communication behavior:

### Reliability Policy
- **Reliable**: All messages are delivered (with retries if needed)
- **Best Effort**: Messages are sent without guarantees of delivery

### Durability Policy
- **Transient Local**: Publishers send recent messages to new subscribers
- **Volatile**: New subscribers only receive future messages

### History Policy
- **Keep Last**: Maintain a fixed number of recent messages
- **Keep All**: Maintain all messages (limited by system resources)

### Depth Policy
Controls how many messages to store in history for keep-last policy.

## Security in ROS 2

Security is a critical feature of ROS 2, addressing concerns not present in ROS 1:

### Authentication
- Identity verification for nodes
- Certificate-based authentication
- Secure identity management

### Access Control
- Permissions for topics, services, and actions
- Role-based access control
- Fine-grained security policies

### Encryption
- Data encryption in transit
- Secure communication channels
- Protection against eavesdropping

## Real-Time Capabilities

ROS 2 supports real-time applications through:

### Real-Time Scheduling
- Integration with operating system schedulers
- Priority-based task execution
- Deterministic behavior

### Memory Management
- Pre-allocated memory pools
- Avoidance of dynamic memory allocation during critical operations
- Predictable memory usage patterns

## Multi-Robot Systems

ROS 2 provides enhanced support for multi-robot scenarios:

### Domain IDs
- Isolated communication domains
- Multiple ROS 2 systems on same network
- Security through isolation

### Discovery Mechanisms
- Automatic node discovery
- Network-level coordination
- Scalable communication

## Launch System

ROS 2 provides a comprehensive launch system:

### Python-based Launch Files
- More powerful and flexible than ROS 1
- Support for conditions and complex logic
- Cross-platform compatibility

### Composable Nodes
- Multiple nodes in single process
- Reduced communication overhead
- Improved performance

## Tools and Ecosystem

ROS 2 includes a rich set of development tools:

### Command Line Tools
- `ros2 run`: Execute nodes
- `ros2 topic`: Interact with topics
- `ros2 service`: Interact with services
- `ros2 action`: Interact with actions
- `ros2 node`: Manage nodes
- `ros2 param`: Manage parameters

### Visualization Tools
- RViz2: 3D visualization environment
- rqt: Graphical user interface framework
- PlotJuggler: Data visualization and analysis

## Migration from ROS 1

Key considerations when migrating from ROS 1:

### Architecture Changes
- DDS-based communication instead of custom protocol
- Improved security model
- Enhanced real-time capabilities

### Code Changes
- Updated client library APIs
- New build system (ament vs catkin)
- Different message definition tools

### Best Practices
- Leverage new QoS policies
- Implement security features
- Use improved launch system

## Applications in Humanoid Robotics

ROS 2 is particularly well-suited for humanoid robotics:

### Sensor Integration
- Multiple sensor types with different QoS requirements
- Real-time sensor processing
- Distributed sensor networks

### Control Systems
- Coordinated multi-joint control
- Real-time control loop requirements
- Safety-critical operations

### AI Integration
- Machine learning pipeline integration
- Perception-action loops
- Distributed AI systems

## Future Developments

The ROS 2 ecosystem continues to evolve:

### Performance Improvements
- Reduced communication overhead
- Optimized message passing
- Better resource utilization

### Additional Language Support
- Expanded client library support
- Improved cross-language compatibility
- Enhanced tooling for all languages

### Standardization Efforts
- Increased industry adoption
- Standardized interfaces
- Improved interoperability

## Summary

ROS 2 represents a fundamental shift in robotics middleware design, focusing on production-ready features like security, real-time support, and multi-robot coordination. The adoption of DDS as the communication backbone provides a robust foundation for scalable robotic systems. Understanding the architecture is crucial for effectively utilizing ROS 2 in humanoid robotics applications. The comprehensive set of communication patterns, QoS policies, security features, and development tools make ROS 2 an ideal choice for complex robotic systems that require reliable, scalable, and secure communication.