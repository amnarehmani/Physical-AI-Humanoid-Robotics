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
---

# Lesson 1: Synchronous Services

## The Request-Response Pattern

In networking, the Client-Server model is ubiquitous. Your web browser (Client) requests a page, and the Google server (Server) responds. In ROS 2, **Services** implement this exact pattern.

Unlike Topics, which are Many-to-Many, Services are typically **One-to-One** (or One-Server-to-Many-Clients). A Service is defined by a pair of messages: one for the Request and one for the Response.

### When to use Services?
*   **Short Duration**: The operation should be quick (calculations, state checks, toggles).
*   **Reliability Required**: You need to know *for sure* that the command was received and processed.
*   **Blocking**: The client often pauses (blocks) until the result is ready.

:::warning
**Do not use Services for long-running tasks.** If a service takes 5 seconds to compute, your robot's main loop might freeze waiting for the answer. Use Actions for long tasks.
:::

## Implementing a Service Server

Let's create a simple node that offers a service to "Add Two Ints". While trivial, it demonstrates the handshake perfectly.

### Step 1: Create the Server Node

Create a file named `simple_service_server.py`.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        # Create the Service
        # name: 'add_two_ints'
        # type: AddTwoInts
        # callback: self.add_two_ints_callback
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service "add_two_ints" is ready.')

    def add_two_ints_callback(self, request, response):
        # This function runs whenever a request is received
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        
        return response # Send response back to client

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Running the Server

1.  Make sure your environment is sourced.
2.  Run the node (assuming you added it to `setup.py` or run directly):
    ```bash
    python3 simple_service_server.py
    ```
    *Output: Service "add_two_ints" is ready.*

## Implementing a Service Client

Now we need a node to call this service.

### Step 3: Create the Client Node

Create a file named `simple_service_client.py`.

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        
        # Async call - returns a Future object
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    
    # Send request 4 + 5
    response = minimal_client.send_request(4, 5)
    minimal_client.get_logger().info(
        f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Execution

With the Server running in Terminal 1, run the Client in Terminal 2:

```bash
python3 simple_service_client.py
```

**Terminal 2 Output:**
`[INFO]: Result of add_two_ints: 9`

**Terminal 1 Output:**
`[INFO]: Incoming request a: 4 b: 5`

## Deep Dive: Sync vs Async

Notice we used `call_async`. Why?
In ROS 2, nodes are typically single-threaded. If you use a synchronous call (`call()`), the client will block the entire thread waiting for a response. If the server is on the same thread (unlikely) or if the network hangs, your robot freezes.

**Best Practice**: Always use `call_async` within a dedicated callback or a separate thread to ensure your robot can still process sensor data (Topics) while waiting for the Service response.

## End-of-Lesson Checklist

- [ ] I can explain the difference between a Topic and a Service.
- [ ] I have successfully run the `AddTwoInts` server and client.
- [ ] I understand why `call_async` is preferred over synchronous calls.
- [ ] I know where to find standard service definitions (`ros2 interface list`).

In the next lesson, we will tackle the more complex but powerful sibling of the Service: The Action.
