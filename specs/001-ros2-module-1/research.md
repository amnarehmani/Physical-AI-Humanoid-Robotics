# Research: Module 1 (ROS 2)

**Status**: Complete
**Date**: 2025-12-08

## 1. Technical Decisions

### ROS 2 Distro Compatibility
- **Decision**: Code will target **Foxy** and **Humble** APIs.
- **Rationale**: These are the most common LTS versions in education. `rclpy` API is stable between them for basic Pub/Sub.
- **Verification**: Checked minimal examples; `create_publisher`, `create_subscription`, and `spin` are identical.

### Python Control Pattern
- **Decision**: Use **Object-Oriented** Node structure (inheriting from `rclpy.node.Node`).
- **Rationale**: Standard practice in ROS 2; avoids global state; easier to test and extend.
- **Alternatives**: Script-style (functions only) - rejected as it promotes bad habits for complex robots.

### URDF Complexity
- **Decision**: Use a **pure XML** snippet (no Xacro yet).
- **Rationale**: Lesson 3 is an *introduction* to the concept. Xacro adds complexity (macros) that obscures the raw XML tag structure students need to learn first.
- **Structure**: `base_link` (torso) -> `neck_joint` -> `head_link`.

## 2. Key Concepts to Cover

### The Computing Graph
- **Metaphor**: "Nervous System".
- **Components**:
    - **Nodes**: Neurons/Organs (Processors).
    - **Topics**: Nerves/Signals (Continuous data).
    - **Services**: Reflexes/Queries (Request-Response).
    - **Actions**: Complex behaviors (Goal-Feedback-Result) - *Mention briefly, detailed later*.

### Implementation details
- **Publisher**: `self.create_publisher(MsgType, 'topic_name', qos_profile)`
- **Subscriber**: `self.create_subscription(MsgType, 'topic_name', callback, qos_profile)`
- **Execution**: `rclpy.init()`, `rclpy.spin(node)`, `rclpy.shutdown()`

## 3. Artifacts & References
- **Official Docs**: https://docs.ros.org/en/humble/
- **PEP 8**: Standard Python style guide.
