---
sidebar_position: 3
---

# Nodes, Topics, Services, Actions

## Core Communication Patterns in ROS 2

ROS 2 provides three primary communication patterns that enable different types of interactions between nodes in a robotic system. Understanding these patterns is crucial for designing effective robotic applications.

## Nodes

A **Node** is the fundamental unit of computation in ROS 2. It's a process that performs computation and can communicate with other nodes. In humanoid robotics, nodes might represent:

- Sensor drivers (camera, LiDAR, IMU)
- Control algorithms (walking controllers, arm controllers)
- Perception systems (object detection, SLAM)
- Planning systems (path planning, task planning)
- User interfaces (command interfaces, visualization)

### Node Characteristics
- Each node has a unique name within the ROS 2 graph
- Nodes can be written in different programming languages
- Nodes can be run on different machines
- Nodes contain publishers, subscribers, services, and actions

## Topics - Publish/Subscribe Pattern

**Topics** implement a publish/subscribe communication pattern where data flows from publishers to subscribers. This is ideal for streaming data like sensor readings or robot state.

### Key Features
- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Many-to-many**: Multiple publishers can publish to a topic, multiple subscribers can subscribe
- **Data-driven**: Communication happens when data is available
- **Real-time friendly**: Low latency for streaming data

### Example in Humanoid Robotics
- Joint state publisher broadcasting current joint angles
- IMU data streaming to multiple perception nodes
- Camera image streams to computer vision nodes
- Robot pose updates for visualization

### Quality of Service (QoS)
ROS 2 provides QoS settings to control how messages are delivered:
- Reliability (reliable vs. best effort)
- Durability (transient local vs. volatile)
- History (keep last N vs. keep all)

## Services - Request/Response Pattern

**Services** implement a request/response communication pattern similar to REST APIs. This is ideal for operations that have a clear input and output with a defined completion.

### Key Features
- **Synchronous**: Client waits for response from server
- **One-to-one**: One service server responds to one client at a time
- **Stateless**: Each request is independent
- **Error handling**: Server can return error responses

### Example in Humanoid Robotics
- Navigation goal request (send destination, get confirmation)
- Calibration service (send command, get calibration results)
- Robot enable/disable service
- Parameter configuration service

## Actions - Goal-Based Pattern

**Actions** are designed for long-running tasks that provide feedback during execution and return a result upon completion. They combine aspects of both topics and services.

### Key Features
- **Long-running**: Designed for operations that take time
- **Feedback**: Provides continuous feedback during execution
- **Goal management**: Can accept/reject goals, cancel running goals
- **Result**: Returns a final result when complete

### Example in Humanoid Robotics
- Walking to a location (feedback on progress, result when reached)
- Manipulation task (feedback on grasp progress, result of success/failure)
- Mapping operation (feedback on coverage, result map)
- Complex trajectory execution

## Comparison and When to Use Each

| Pattern | Communication | Use Case | Example |
|---------|---------------|----------|---------|
| Topic | Publish/Subscribe | Streaming data | Sensor data, robot state |
| Service | Request/Response | Single request/response | Navigation goal, calibration |
| Action | Goal/Feedback/Result | Long-running with feedback | Walking, manipulation |

## Implementation Example

Here's a conceptual example of how these patterns work together in a humanoid robot:

```
Camera Node (Publisher)
    ↓ (Image Topic)
Computer Vision Node
    ↓ (Detection Topic)
Planning Node
    ↓ (Service Request)
Navigation Service Server
    ↓ (Action Goal)
Walking Controller (Action Server)
    ↓ (Feedback/Result)
Planning Node
    ↓ (Robot State Topic)
Visualization Node (Subscriber)
```

## Best Practices

1. **Use Topics** for continuous data streams and state updates
2. **Use Services** for discrete operations with clear input/output
3. **Use Actions** for complex operations that take time and need monitoring
4. **Consider QoS settings** based on your real-time requirements
5. **Design message types** that are efficient and meaningful
6. **Handle connection and disconnection** events gracefully

## Integration with Physical AI

In Physical AI applications, these communication patterns enable:
- Real-time sensor data integration for embodied intelligence
- Coordination between perception, planning, and control systems
- Human-robot interaction through various interface modalities
- Safe and robust operation through proper error handling

The next section will explore how Python agents can bridge to ROS 2 controllers using the rclpy library.