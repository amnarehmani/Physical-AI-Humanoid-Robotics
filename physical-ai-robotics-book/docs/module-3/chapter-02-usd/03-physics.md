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

## Physics Schemas

USD is just data. To make it fall, we must apply **Physics Schemas** (APIs).
*   `UsdPhysicsRigidBodyAPI`: Makes it move.
*   `UsdPhysicsCollisionAPI`: Makes it solid.
*   `UsdPhysicsMassAPI`: Gives it weight.

## Applying Physics via Code

```python
from pxr import UsdPhysics

# 1. Select the Prim
cube = stage.GetPrimAtPath("/World/MyCube")

# 2. Make it a Rigid Body
rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(cube)
rigid_body_api.CreateRigidBodyEnabledAttr(True)

# 3. Add Collision
collision_api = UsdPhysics.CollisionAPI.Apply(cube)
collision_api.CreateCollisionEnabledAttr(True)
```

## Physics Materials

Just like visual materials, we have Physics Materials (Friction, Restitution/Bounciness).
You create a `PhysicsMaterial` Prim and bind it to the collision geometry.

```python
# Create Material
mat_path = "/World/PhysicsMaterials/Rubber"
UsdShade.Material.Define(stage, mat_path)
# ... set friction to 1.0 ...

# Bind to Cube
UsdShade.MaterialBindingAPI(cube).Bind(mat_prim)
```

## Debugging Physics

In Isaac Sim, enable: `Physics -> Debug -> Show Colliders`.
You will see purple lines.
*   **Purple Box**: The collision shape.
*   **Visible Mesh**: The render shape.
If the purple box is missing, the object will fall through the floor.

## End-of-Lesson Checklist

- [ ] I can apply the Rigid Body API to a Prim.
- [ ] I can apply the Collision API to a Prim.
- [ ] I verify collision shapes using the Physics Debugger.
- [ ] I understand that Visual and Physics geometries are separate in USD.
