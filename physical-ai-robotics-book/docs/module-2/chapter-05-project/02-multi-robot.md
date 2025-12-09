---
id: m2-ch5-multi-robot
title: "Lesson 2: Multi-Robot Launch"
sidebar_label: "Lesson 2: Multi-Robot"
description: "Spawning robots with namespaces."
keywords:
  - namespaces
  - multi-robot
  - tf_prefix
  - launch
---

# Lesson 2: Multi-Robot Launch

## The Namespace Problem

If you spawn two robots, they both try to publish to `/scan` and `/odom`. The data gets mixed up.
We need **Namespaces**: `/bot1/scan`, `/bot2/scan`.

## TF Prefixing

We also have a TF problem. Both robots have a `base_link`. The TF tree cannot have two frames with the same name.
We must prefix the frames: `bot1/base_link`, `bot2/base_link`.

## The Launch Strategy

We create a "Group Action" in our launch file for each robot.

```python
# multi_bot.launch.py
def generate_launch_description():
    
    # Robot 1
    bot1 = GroupAction([
        PushRosNamespace('bot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('spawn_robot.launch.py'),
            launch_arguments={'x': '0', 'y': '0', 'name': 'bot1'}.items()
        )
    ])

    # Robot 2
    bot2 = GroupAction([
        PushRosNamespace('bot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('spawn_robot.launch.py'),
            launch_arguments={'x': '2', 'y': '0', 'name': 'bot2'}.items()
        )
    ])
    
    return LaunchDescription([bot1, bot2])
```

## Configuring the Bridge

The bridge also needs to know about namespaces.
```yaml
- topic_name: "/bot1/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
```
Or use wildcards (if supported by your bridge version) or generate the YAML dynamically in the launch file.

## End-of-Lesson Checklist

- [ ] I understand why namespaces are required for multi-robot systems.
- [ ] I have modified my spawn script to accept a `name` argument.
- [ ] I have created a launch file that spawns two distinct robots.
- [ ] I have verified via `ros2 topic list` that topics are separated (e.g., `/bot1/...`).
