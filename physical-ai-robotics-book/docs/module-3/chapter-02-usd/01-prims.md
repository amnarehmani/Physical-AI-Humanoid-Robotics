---
id: m3-ch2-prims
title: "Lesson 1: Prims and Properties"
sidebar_label: "Lesson 1: Prims"
description: "Navigating the USD Stage hierarchy."
keywords:
  - usd
  - prim
  - attribute
  - python
  - omni
---

# Lesson 1: Prims and Properties

## The Stage and The Prim

The **Stage** is the world.
A **Prim** (Primitive) is any node in the tree (a Mesh, a Light, a Joint, a Material).
A **Property** is data attached to a Prim (position, color, mass).

## Accessing the Stage

In Isaac Sim's Script Editor:

```python
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Usd, UsdGeom

stage = get_current_stage()
```

## Creating a Prim

Let's create a Cube programmatically.

```python
# Define the path
prim_path = "/World/MyCube"

# Define the type (Cube)
cube_prim = UsdGeom.Cube.Define(stage, prim_path)

# Set a property (Size)
cube_prim.GetSizeAttr().Set(2.0)
```

## Traversing the Stage

To find all robots in the scene:

```python
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Xform):
        print(f"Found object: {prim.GetPath()}")
```

## Relationships

Prims can relate to other Prims. A "Material Binding" is a relationship.
If you drag a Red material onto the Cube, USD creates a `rel material:binding` property on the Cube pointing to `/World/Looks/RedMaterial`.

## End-of-Lesson Checklist

- [ ] I can get the current `UsdStage` object in Python.
- [ ] I can create a basic Prim (Cube/Sphere) using code.
- [ ] I understand that everything in the Stage window is a Prim.
- [ ] I have used `Traverse()` to iterate through the scene.
