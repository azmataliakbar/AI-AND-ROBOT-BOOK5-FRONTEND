---
sidebar_position: 2
title: "Chapter 2: Nodes and Topics"
---

# Nodes and Topics in ROS 2

## Understanding Nodes

In ROS 2, a **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Multiple nodes work together to form a complete robotic application.

### Creating a Node

A node in ROS 2 typically:

- Performs a specific task
- Communicates with other nodes
- Can be written in different programming languages
- Runs independently of other nodes

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
```

## Topics and Message Passing

**Topics** are named buses over which nodes exchange messages. A node can publish messages to a topic or subscribe to messages from a topic.

### Publisher-Subscriber Pattern

- **Publishers** send data to a topic
- **Subscribers** receive data from a topic
- Multiple publishers and subscribers can use the same topic
- Communication is asynchronous

## Quality of Service (QoS)

ROS 2 introduces Quality of Service profiles that allow you to specify requirements for:

- Reliability (reliable vs. best-effort delivery)
- Durability (latching behavior)
- History (how many messages to keep)
- Rate limits

This is a key improvement over ROS 1 that allows ROS 2 to be used in more demanding applications.

In the next chapter, we'll explore services and actions in ROS 2.