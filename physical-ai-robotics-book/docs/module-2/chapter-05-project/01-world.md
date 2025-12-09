---
id: m2-ch5-world
title: "Lesson 1: Building the World"
sidebar_label: "Lesson 1: World Building"
description: "Creating SDF worlds with static and dynamic objects."
keywords:
  - sdf
  - world
  - gazebo
  - assets
---

# Lesson 1: Building the World

## SDF vs URDF

*   **URDF**: For Robots (Kinematics, Joints).
*   **SDF (Simulation Description Format)**: For Worlds (Lighting, Physics, Environment).

While you *can* use URDF for a world, SDF is native to Gazebo and supports features like "Actors" (moving humans) and complex lighting.

## Creating a World File

Create `warehouse.sdf`:

```xml
<sdf version="1.6">
  <world name="warehouse">
    <!-- 1. Sun & Ground -->
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- 2. Shelves -->
    <include>
      <name>shelf_1</name>
      <pose>2 2 0 0 0 0</pose>
      <uri>model://aws_robomaker_warehouse_shelfF_01</uri>
    </include>

    <!-- 3. Physics Settings -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
  </world>
</sdf>
```

## Using Fuel (Asset Library)

Gazebo Fuel is an online repository of models.
You can drag-and-drop models inside the Gazebo GUI, then save the world.
**Pro Tip**: Download models locally if you plan to run without internet.

## Adding Dynamic Obstacles

A static world is boring. Let's add a physics-enabled box that can be pushed.

```xml
<model name="cardboard_box">
  <pose>1 0 0.5 0 0 0</pose>
  <link name="link">
    <inertial><mass>1.0</mass></inertial>
    <collision name="col"><geometry><box size="0.5 0.5 0.5"/></geometry></collision>
    <visual name="vis"><geometry><box size="0.5 0.5 0.5"/></geometry></visual>
  </link>
</model>
```

## End-of-Lesson Checklist

- [ ] I have created a `.sdf` world file.
- [ ] I have added lighting and a ground plane.
- [ ] I have populated the world with static assets (shelves).
- [ ] I have verified that physics works by knocking over a dynamic object.
