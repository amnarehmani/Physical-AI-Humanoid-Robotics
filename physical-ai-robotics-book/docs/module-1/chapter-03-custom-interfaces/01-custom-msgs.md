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
  - build system
---

# Lesson 1: Defining Custom Messages

## 1. Introduction

In previous chapters, we used `std_msgs/String` to send text and `example_interfaces` for our services. While convenient for tutorials, real robots rarely speak in simple strings.

A robot doesn't just say "Face Detected." It says: "Face #42 detected at (X,Y) with 95% confidence."
Trying to shove all this into a String is messy (parsing JSON at 100Hz is slow). Sending three separate messages (ID, Pos, Confidence) introduces synchronization bugs (did this ID match that Position?).

The solution is **Custom Interfaces**. We will define a structured data type that perfectly matches our domain needs.

## 2. Conceptual Understanding: The Data Contract

**Intuition**:
Think of a Custom Message as a **Form** you fill out.
*   **Standard Message (`String`)**: A blank sheet of paper. You can write anything, but the receiver has to read your handwriting.
*   **Custom Message (`FaceDetection`)**: A printed form with boxes for "ID", "Name", and "Coordinates". The receiver knows exactly where to look for the data.

**Mechanism**:
ROS 2 uses an **IDL (Interface Definition Language)** to define these forms.
1.  **Definition**: You write a `.msg` file (language neutral).
2.  **Generation**: The build system (CMake) reads this file and generates Python classes (`module.py`) and C++ headers (`module.hpp`).
3.  **Usage**: You import these classes just like any other library.

## 3. System Perspective: The Build Pipeline

Creating a custom message is more involved than writing a Python script because it requires **Compilation**.

```mermaid-text
[Developer]
    |
    v
(1) Write "MyMessage.msg"
    |
    v
[CMake Build System (colcon)]
    |
    +---> [rosidl_generator_cpp] ---> Generates .hpp files (C++)
    |
    +---> [rosidl_generator_py]  ---> Generates .py files (Python)
    |
    v
[Your Node] <--- Imports generated code
```

## 4. Practical Example: The "Target Coordinates" Message

We will create a message to represent a detected object in 3D space.

### Step 1: The Package
It is best practice to keep interfaces in a separate package to avoid circular dependencies.
```bash
ros2 pkg create --build-type ament_cmake my_robot_interfaces
mkdir -p my_robot_interfaces/msg
```

### Step 2: The Definition
Create `my_robot_interfaces/msg/TargetCoordinates.msg`.

```text
# Unique ID of the target
int32 id

# Descriptive name (e.g. "Coffee Cup")
string name

# Position in 3D space (Nest standard messages!)
geometry_msgs/Point position

# Detection confidence (0.0 - 1.0)
float32 confidence
```

### Step 3: The Build Configuration
This is the tricky part. We must tell CMake to process this file.

**package.xml**:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>geometry_msgs</depend>
```

**CMakeLists.txt**:
```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCoordinates.msg"
  DEPENDENCIES geometry_msgs
)
```

### Step 4: Compile and Verify
```bash
colcon build --packages-select my_robot_interfaces
source install/setup.bash
ros2 interface show my_robot_interfaces/msg/TargetCoordinates
```

## 5. Engineering Insights: Build System Nuances

*   **Why a separate package?**: If `Node A` uses `Message B`, and `Message B` is defined inside `Node A`'s package, you often get build errors. Separating `my_robot_interfaces` allows both `Node A` and `Node B` to depend on the *messages* without depending on each other's *code*.
*   **The "Clean" Build**: If ROS 2 acts like it can't find your message even after you fixed a typo, delete the `build/` and `install/` folders and rebuild. The code generator sometimes caches old definitions.
*   **Nesting**: Always reuse standard messages (`geometry_msgs/Point`, `std_msgs/Header`) inside your custom messages. Don't reinvent `x, y, z` if a standard type exists. This makes your code compatible with other tools (like Rviz).

## 6. Summary

You have learned to extend the language of ROS 2.
1.  **Define** data structures in `.msg` files.
2.  **Configure** `CMakeLists.txt` to generate code.
3.  **Compile** the package to make it available.

Now that we have a structured way to represent a "Target," we need to know *where* that target is. Is it relative to the camera? The floor? The gripper? For that, we need **TF2 Transforms**.