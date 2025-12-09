---
id: m1-ch4-debugging
title: "Lesson 2: Debugging & Diagnostics"
sidebar_label: "Lesson 2: Debugging"
description: "Using ros2 doctor, rqt, and logs."
keywords:
  - ros2
  - debugging
  - rqt
  - doctor
  - logging
---

# Lesson 2: Debugging & Diagnostics

## ROS 2 Doctor

When things go wrong, call the doctor.

```bash
ros2 doctor
```

This command checks:
*   **Platform**: OS version, Kernel, CPU load.
*   **Network**: Multicast support, Interface stats.
*   **ROS 2**: Distribution, Middleware (RMW) mismatch.

If it finds issues, it prints a warning.
`UserWarning: The RMW implementation 'rmw_cyclonedds_cpp' is active but 'rmw_fastrtps_cpp' is also installed.`

## The RQt Suite

RQt is a plugin-based GUI.

1.  **rqt_graph**: Visualizes the node diagram.
    *   *Use*: "Why is my subscriber not receiving data?" (Maybe the lines aren't connected).
2.  **rqt_console**: Advanced log viewer.
    *   *Use*: Filter logs by Node, Level (Error/Warn), or Time.
3.  **rqt_plot**: Real-time graphing.
    *   *Use*: Plotting `/joint_states/position[0]` to see if a motor is oscillating.

## Advanced Logging in Code

Don't just print strings. Use the logger features.

```python
# Throttle: Print max once per second (prevents spam)
self.get_logger().info('Sensor Active', throttle_duration_sec=1.0)

# Skip First: Ignore initialization noise
self.get_logger().warn('Low Battery', skip_first=true)

# Expression: Only log if x > 10
self.get_logger().error('Overheat!', once=True) # Only print the first time it happens
```

## End-of-Lesson Checklist

- [ ] I have run `ros2 doctor` and interpreted the report.
- [ ] I have used `rqt_graph` to identify a disconnected topic.
- [ ] I can use `throttle_duration_sec` in my Python nodes to reduce log noise.
- [ ] I have plotted a value using `rqt_plot` (or `foxglove`).
