# Data Model: Module 1 Content Structure

## 1. Hierarchy

The content follows the Docusaurus hierarchy: `Module` -> `Lesson` -> `Section`.

### Module: The Robotic Nervous System
- **ID**: `module-1`
- **Slug**: `/modules/ros2-nervous-system`
- **Metadata**:
    - `title`: "The Robotic Nervous System (ROS 2)"
    - `difficulty`: "Beginner"
    - `prerequisites`: ["Python Basics", "Linux Basics"]

## 2. Lesson Definitions

### Lesson 1: Architecture
- **Filename**: `lesson-1-architecture.md`
- **Title**: "The Computing Graph"
- **Key Concepts**: Nodes, Topics, Services, Messages.
- **Metaphor**: The Biological Nervous System.

### Lesson 2: Control
- **Filename**: `lesson-2-control.md`
- **Title**: "Controlling the Body (Python)"
- **Key Concepts**: `rclpy`, Publisher, Subscriber, Executor.
- **Code Artifacts**:
    - `publisher.py`: Sends commands.
    - `subscriber.py`: Receives commands.

### Lesson 3: Structure
- **Filename**: `lesson-3-urdf.md`
- **Title**: "Defining the Body (URDF)"
- **Key Concepts**: Links, Joints, Visuals, Collisions, XML.
- **Code Artifacts**:
    - `simple_humanoid.urdf`: XML description of Torso+Head.

## 3. Frontmatter Schema
All markdown files must include:
```yaml
---
id: [lesson-id]
title: [Lesson Title]
sidebar_label: [Short Title]
description: [SEO Description]
keywords: [ros2, python, robotics, urdf]
---
```
