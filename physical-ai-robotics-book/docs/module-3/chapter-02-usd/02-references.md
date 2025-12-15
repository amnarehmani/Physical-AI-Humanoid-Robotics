---
id: m3-ch2-references
title: "Lesson 2: References and Payloads"
sidebar_label: "Lesson 2: References"
description: "Composing scenes from multiple assets."
keywords:
  - usd
  - reference
  - payload
  - composition
---

# Lesson 2: References and Payloads

## 1. Introduction

The power of USD lies in **Composition**.
Instead of building a robot from scratch every time, we build a library of parts (Wheels, Sensors, Motors) and compose them together.

## 2. Conceptual Understanding: The Reference Arc

A **Reference** is a pointer.
*   **Asset**: `wheel.usd` (Contains the mesh and physics of a wheel).
*   **Robot**: `robot.usd` (References `wheel.usd` 4 times).

If we update `wheel.usd` to have better tire treads, `robot.usd` updates automatically.

### Reference vs Payload
*   **Reference**: "I need this data to function." (Always loaded).
*   **Payload**: "I might need this data." (Deferred loading).
    *   Use Payloads for heavy background assets (buildings, trees). You can unload them to save RAM while working on the robot.

## 3. Implementation: Python References

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# The path to the Asset
asset_path = "omniverse://localhost/Assets/Robots/Carter/carter_v1.usd"

# 1. Reference it
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Carter_1")

# 2. Reference it AGAIN (Instancing)
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Carter_2")
```

Now we have two robots. They share the same underlying data on disk.

## 4. Engineering Insights: Deltas (Overrides)

We can modify a referenced object. This creates a **Delta**.

```python
from pxr import UsdGeom, Gf

# Get the second robot
prim = stage.GetPrimAtPath("/World/Carter_2")
xform = UsdGeom.XformCommonAPI(prim)

# Move it (This overrides the position from the original file)
xform.SetTranslate(Gf.Vec3d(2.0, 0.0, 0.0))
```

The original `carter_v1.usd` says "Position = 0,0,0".
Our stage says "For Carter_2, Position = 2,0,0".
USD composes these opinions to place the robot.

## 5. Summary

References allow us to build modular, reusable assets.
*   **References** link files together.
*   **Payloads** allow lazy loading for performance.
*   **Deltas** allow us to customize instances without breaking the original asset.

In the next lesson, we will learn how to make these static meshes move using the **Physics API**.