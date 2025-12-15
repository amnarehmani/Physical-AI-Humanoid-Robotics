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

## 1. Introduction

A robot needs a place to live. In Module 1, we used an empty void. Now, we will build a warehouse.
We use **SDF** (Simulation Description Format), which is Gazebo's native language for environments. Unlike URDF, SDF allows us to define lighting, skyboxes, and multiple disconnected objects.

## 2. Conceptual Understanding: The Scene Graph

An SDF world is a tree structure.

```text
      [ World ]
          |
      +---+---+----------------+
      |       |                |
   [ Sun ] [ Ground ]      [ Models ]
                               |
                        +------+------+
                        |             |
                    [ Static ]    [ Dynamic ]
                    (Shelves)      (Boxes)
```

*   **Include**: References external models (like library imports).
*   **Pose**: Position ($x, y, z$) and Orientation ($r, p, y$) in the world frame.

## 3. Implementation: The Warehouse SDF

Create a file `code/module-2/worlds/warehouse.sdf`.

### 3.1 Basic Environment
We always need light and gravity.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="warehouse_world">
    <!-- Physics Engine Config -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Sun and Ground -->
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri></include>
```

### 3.2 Adding Assets (Shelves)
We can pull high-quality assets from **Gazebo Fuel**.

```xml
    <!-- A Warehouse Shelf -->
    <include>
      <name>Shelf_A</name>
      <pose>2 0 0 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse Shelf</uri>
    </include>

    <!-- Another Shelf -->
    <include>
      <name>Shelf_B</name>
      <pose>2 3 0 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse Shelf</uri>
    </include>
```

### 3.3 Dynamic Objects (The "Ouch" Factor)
Let's add a heavy box that the robot can push (or crash into).

```xml
    <model name="heavy_box">
      <pose>0 2 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>10.0</mass>
          <inertia ixx="1.6" ixy="0" ixz="0" iyy="1.6" iyz="0" izz="1.6"/>
        </inertial>
        <collision name="col">
          <geometry><box size="1 1 1"/></geometry>
        </collision>
        <visual name="vis">
          <geometry><box size="1 1 1"/></geometry>
          <material><ambient>1 0 0 1</ambient></material> <!-- Red -->
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## 4. Engineering Insights: Asset Caching

When you include a URL from Fuel, Gazebo downloads it. This can be slow.
**Best Practice**: Download assets once and store them in `~/.gazebo/models` or a local `models/` folder in your package. Then use `model://my_shelf` instead of the URL. This ensures your simulation works offline and loads instantly.

## 5. Summary

We have constructed a physical space.
*   It has **Physics Rules** (Gravity, Time Step).
*   It has **Static Infrastructure** (Shelves).
*   It has **Dynamic Hazards** (Boxes).

In the next lesson, we will populate this ghost town with multiple robots.