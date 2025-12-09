---
id: m1-ch3-msgs
title: "Lesson 1: Defining Custom Messages"
sidebar_label: "Lesson 1: Custom Messages"
description: "Creating .msg and .srv files for custom data structures."
keywords:
  - ros2
  - msg
  - srv
  - cmake
---

# Lesson 1: Defining Custom Messages

## Why Custom Messages?

ROS 2 comes with `std_msgs` (String, Int32), `geometry_msgs` (Point, Pose), and `sensor_msgs` (Image, LaserScan).
However, mixing and matching these often leads to semantic confusion. sending a `Vector3` to represent "RGB Color" is bad practice. Instead, define a `Color` message.

## Creating an Interface Package

It is best practice to keep message definitions in a separate package (e.g., `my_robot_interfaces`) to avoid circular dependencies.

1.  **Create the package**:
    ```bash
    ros2 pkg create --build-type ament_cmake my_robot_interfaces
    ```
2.  **Create directories**:
    ```bash
    mkdir msg srv action
    ```

## Defining a Message (.msg)

Create `msg/TargetCoordinates.msg`:

```text
# ID of the detected target
int32 id

# Name of the target
string name

# Position in space
geometry_msgs/Point position

# Confidence score (0.0 to 1.0)
float32 confidence
```

Notice we can nest other messages (`geometry_msgs/Point`) inside ours.

## Defining a Service (.srv)

Create `srv/SetLedColor.srv`:

```text
# Request
string color
bool blink
---
# Response
bool success
string message
```

## Configuring Build System

To turn these text files into Python/C++ code, we edit `CMakeLists.txt` and `package.xml`.

### package.xml
Add these lines:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>geometry_msgs</depend> <!-- Because we used it -->
```

### CMakeLists.txt
Before `ament_package()`:

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCoordinates.msg"
  "srv/SetLedColor.srv"
  DEPENDENCIES geometry_msgs
)
```

## Compilation

Run `colcon build --packages-select my_robot_interfaces`.
Source the workspace: `source install/setup.bash`.

## Verify

Check if ROS sees your new type:
```bash
ros2 interface show my_robot_interfaces/msg/TargetCoordinates
```
*Output should match your definition.*

## Using it in Python

```python
from my_robot_interfaces.msg import TargetCoordinates

msg = TargetCoordinates()
msg.id = 1
msg.name = "Coffee Cup"
msg.confidence = 0.95
publisher.publish(msg)
```

## End-of-Lesson Checklist

- [ ] I have created a dedicated package for interfaces.
- [ ] I have written a `.msg` file combining primitives and other messages.
- [ ] I have successfully compiled the package using `colcon build`.
- [ ] I can import the new message type in a Python script.
