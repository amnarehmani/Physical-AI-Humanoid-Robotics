---
id: m1-ch2-services
title: "Lesson 1: Synchronous Services"
sidebar_label: "Lesson 1: Services"
description: "Implementing Request-Response patterns with ROS 2 Services."
keywords:
  - ros2
  - services
  - client
  - server
  - python
  - synchronous
  - blocking
---

# Lesson 1: Synchronous Services

## 1. Introduction

In the previous chapter, we built a robot that could stream data. But what if we need to ask our robot a specific question? "Are you calibrated?" "What is the battery level?" "Spawn a cube at coordinates (X, Y, Z)."

Topics are ill-suited for this. If you publish "Spawn Cube" to a topic, you have no immediate way of knowing if it worked, or if the message was even received. You need a **transaction**.

This lesson introduces **Services**, the Request-Response mechanism of ROS 2. Mastering services is essential for building state machines, coordinating complex startups, and managing system configuration.

## 2. Conceptual Understanding: The Handshake

**Intuition**:
*   **Topic**: A radio host speaking. They don't know if you heard them.
*   **Service**: A phone call. You dial (Request), they pick up, you ask a question, they answer (Response), and you hang up.

**Mechanism**:
A Service is defined by a pair of messages:
1.  **Request**: The data sent by the Client (e.g., `a=4, b=5`).
2.  **Response**: The data returned by the Server (e.g., `sum=9`).

Unlike Topics, Services are **Synchronous** by default. The Client usually waits (blocks) until the Server responds. This "blocking" behavior is powerful but dangerous.

## 3. System Perspective: The Transaction Flow

Let's visualize the lifecycle of a Service call.

```mermaid-text
      [Client Node]                       [Server Node]
           |                                    |
           |  (1) Create Request object         |
           |      req.a=4, req.b=5              |
           |                                    |
           |  (2) Send Request (DDS)            |
           |----------------------------------->|
           |                                    |
           |                                    | (3) Trigger Callback
           |      (WAITING / BLOCKING)          |     process_request(req)
           |                                    |     res.sum = 9
           |                                    |
           |                                    | (4) Send Response
           |<-----------------------------------|
           |                                    |
           |  (5) Unblock / Future Complete     |
           |      print(res.sum)                |
           |                                    |
```

## 4. Practical Example: The Adder

We will implement a classic "Add Two Ints" server. While simple, it perfectly demonstrates the blocking nature of the pattern.

### 4.1 The Server
This node offers the service. It sits idle until called.

```python title="code/module-1/simple_service_server.py"
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # Create Service: type, name, callback
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service "add_two_ints" is ready.')

    def add_two_ints_callback(self, request, response):
        # 1. Process Data
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        
        # 2. Return Response
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 The Client
This node sends the request.

```python title="code/module-1/simple_service_client.py"
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 1. Check for Server availability
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        
        # 2. Send Async Request
        # Returns a "Future" object that will eventually hold the result
        self.future = self.cli.call_async(self.req)
        
        # 3. Wait for Future
        # Note: We spin THIS node until the future completes
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    
    response = minimal_client.send_request(4, 5)
    minimal_client.get_logger().info(f'Result: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Engineering Insights: The Danger of Blocking

In ROS 2, nodes are typically **Single Threaded**.

*   **The Trap**: If you call `client.call()` (synchronous), the *entire node freezes* while waiting for the response.
*   **The Consequence**: If that same node receives `EmergencyStop` on a Topic while frozen, it will **not process the stop command** until the service returns. Your robot crashes.
*   **The Solution**: Always use `call_async()`. While the request is pending, the node yields control back to the executor, allowing other callbacks (timers, topics) to run.

## 6. Summary

Services provide the transactional capability missing from Topics.
1.  **Servers** provide logic on demand.
2.  **Clients** consume logic and wait for results.
3.  **Async** calls are mandatory for safety in single-threaded systems.

Services are great for quick questions ("What time is it?"). But what if the question takes 10 minutes to answer ("Go to the kitchen")? For that, we need **Actions**, which we cover next.