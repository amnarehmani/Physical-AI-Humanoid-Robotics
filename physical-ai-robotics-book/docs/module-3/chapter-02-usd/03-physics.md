---
id: m3-ch2-physics
title: "Lesson 3: USD Physics"
sidebar_label: "Lesson 3: USD Physics"
description: "Applying rigid body dynamics to USD Prims."
keywords:
  - usd
  - physics
  - rigid body
  - collider
  - mass
---

# Lesson 3: USD Physics

## 1. Introduction

A standard USD mesh is a ghost. It has no mass, no solidity, and gravity ignores it.
To make it real, we must **Apply Schemas**.
In USD, "being a Rigid Body" isn't a type; it's a trait (API) you apply to an existing Prim.

## 2. Conceptual Understanding: The API Schema

Think of API Schemas as "Capabilities" in a game.
*   **Mesh Prim**: "I am a shape."
*   **Apply RigidBodyAPI**: "Now I have mass and velocity."
*   **Apply CollisionAPI**: "Now I can hit things."

This separation allows us to turn physics on/off dynamically.

## 3. Implementation: Applying Physics

```python
from pxr import UsdPhysics

# 1. Get the Prim
cube_prim = stage.GetPrimAtPath("/World/MyCube")

# 2. Apply Rigid Body (Motion)
rb_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
rb_api.CreateRigidBodyEnabledAttr(True)
rb_api.CreateKinematicEnabledAttr(False) # True = Animated by script, False = Physics

# 3. Apply Collision (Solidity)
col_api = UsdPhysics.CollisionAPI.Apply(cube_prim)
col_api.CreateCollisionEnabledAttr(True)
col_api.CreateSimulationOwnerAttr(None)
```

## 4. Physics Materials

Friction and bounciness are defined in a **Physics Material** and bound to the prim.

```python
from pxr import UsdShade, UsdPhysics

# Define Material
mat_path = "/World/Materials/Rubber"
mat_prim = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(mat_path, "Material"))
mat_prim.CreateStaticFrictionAttr(1.0)
mat_prim.CreateDynamicFrictionAttr(1.0)
mat_prim.CreateRestitutionAttr(0.5) # Bounciness

# Bind Material to Cube
binding_api = UsdShade.MaterialBindingAPI.Apply(cube_prim)
binding_api.Bind(
    stage.GetPrimAtPath(mat_path), 
    UsdShade.Tokens.physics # Bind specifically for physics purpose
)
```

## 5. Engineering Insights: Mesh Approximation

Using a 100k polygon mesh for collision is slow.
Isaac Sim (PhysX) can auto-compute approximations.
*   **Convex Hull**: Wraps the mesh in a tight skin. Good for general objects.
*   **Bounding Box**: Fastest. Good for crates.
*   **Mesh Decomposition (V-HACD)**: Breaks complex concave shapes (like a cup) into multiple convex hulls.

You configure this in the `MeshCollisionAPI`.

## 6. Summary

We have turned a USD file into a physical simulation.
*   **Prims** hold the data.
*   **References** assemble the scene.
*   **Physics APIs** make it move.

This concludes our deep dive into USD. In the next chapter, we will use **Replicator** to generate synthetic data for AI training.