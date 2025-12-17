---
sidebar_position: 4
---

# Python Agents Bridging to ROS 2

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2, providing Python APIs that allow developers to create ROS 2 nodes, publish and subscribe to topics, provide and use services, and interact with actions. It serves as the bridge between Python-based AI/ML agents and the ROS 2 robotic middleware.

## Why Python for AI Integration

Python has become the de facto language for AI and machine learning development due to its:
- Rich ecosystem of scientific computing libraries (NumPy, SciPy)
- Leading AI frameworks (TensorFlow, PyTorch, scikit-learn)
- Ease of prototyping and experimentation
- Strong community support in the AI/ML domain

This makes `rclpy` essential for integrating AI capabilities with robotic systems.

## Basic Node Structure

A minimal ROS 2 Python node follows this structure:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Publishers and Subscribers

### Publisher Example
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIOutputPublisher(Node):
    def __init__(self):
        super().__init__('ai_output_publisher')
        self.publisher = self.create_publisher(String, 'ai_decisions', 10)

    def publish_decision(self, decision):
        msg = String()
        msg.data = decision
        self.publisher.publish(msg)
```

### Subscriber Example
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Process the image with AI algorithm
        processed_result = self.ai_process_image(msg)
        # Do something with the result
```

## Service Clients and Servers

### Service Server Example
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Service Client Example
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Action Clients and Servers

### Action Client Example
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')
```

## Integration with AI/ML Frameworks

### Example: Integrating TensorFlow with ROS 2
```python
import rclpy
from rclpy.node import Node
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TensorFlowNode(Node):
    def __init__(self):
        super().__init__('tensorflow_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, 'ai_output', 10)

        # Load pre-trained model
        self.model = tf.keras.models.load_model('path/to/model')
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess image for model
        input_tensor = tf.convert_to_tensor(cv_image)
        input_tensor = tf.expand_dims(input_tensor, 0)  # Add batch dimension

        # Run inference
        predictions = self.model(input_tensor)

        # Process results and publish
        result = self.process_predictions(predictions)
        self.publish_result(result)
```

## Best Practices for Python Agents

1. **Use appropriate QoS settings** for your application's requirements
2. **Handle exceptions gracefully** to maintain system stability
3. **Use threading appropriately** to avoid blocking the main ROS loop
4. **Manage memory efficiently** when processing large data (images, point clouds)
5. **Log appropriately** for debugging and monitoring
6. **Follow ROS 2 naming conventions** for nodes, topics, and services

## Error Handling and Robustness

```python
import rclpy
from rclpy.node import Node
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        # Setup with error handling
        try:
            self.setup_components()
        except Exception as e:
            self.get_logger().error(f'Failed to setup node: {e}')
            self.get_logger().error(traceback.format_exc())

    def setup_components(self):
        # Initialize publishers, subscribers, etc.
        pass

    def safe_callback(self, msg):
        try:
            # Process message
            result = self.process_message(msg)
            self.publish_result(result)
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')
            self.get_logger().error(traceback.format_exc())
```

## Performance Considerations

- **Batch processing**: Process multiple messages together when possible
- **Threading**: Use separate threads for CPU-intensive AI processing
- **Memory management**: Be mindful of memory usage with large data like images
- **Timing**: Consider the real-time requirements of your robotic application

The next section will cover URDF (Unified Robot Description Format) and how it's used for humanoid robot description in ROS 2 systems.