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

## 1. Introduction

The **Prim** (Primitive) is the atom of the USD universe.
*   Is a robot a Prim? Yes.
*   Is a wheel a Prim? Yes.
*   Is the light source a Prim? Yes.

Every Prim has a **Path** (like a file path) and a **Type**.
Example: `/World/Robot/Arm/Hand` (Type: Xform).

## 2. Conceptual Understanding: The Stage Graph

The USD Stage is a directed acyclic graph (DAG).

```text
/ (Root)
├── World (Xform)
│   ├── Cube (Cube)
│   │   └── size: 2.0 (Attribute)
│   ├── Light (DistantLight)
│   │   └── intensity: 1000 (Attribute)
│   └── Camera (Camera)
```

**Attributes** hold the data (numbers, strings).
**Relationships** connect Prims (e.g., binding a Material to a Mesh).

## 3. Implementation: Python API

Isaac Sim provides the `pxr` (Pixar) library to manipulate USD directly.

### 3.1 Getting the Stage
```python
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Usd, UsdGeom

# Get the active stage (the open file)
stage = get_current_stage()
```

### 3.2 Creating a Prim
Let's create a visual shape.

```python
# 1. Define the Prim Type and Path
path = "/World/MyCube"
cube_geom = UsdGeom.Cube.Define(stage, path)

# 2. Set Attributes
# Set size (edge length)
cube_geom.GetSizeAttr().Set(1.0) 

# Set position (Translation) using Xform Common API
from pxr import Gf # Graphics Foundation (Math library)
xform = UsdGeom.XformCommonAPI(cube_geom)
xform.SetTranslate(Gf.Vec3d(0.0, 2.0, 0.5))
```

### 3.3 Reading Attributes
```python
prim = stage.GetPrimAtPath("/World/MyCube")
if prim.IsValid():
    print(prim.GetName())
    # Read custom attributes
    attr = prim.GetAttribute("size")
    print(attr.Get())
```

## 4. Engineering Insights: Typed vs Generic

In Python, `stage.GetPrimAtPath()` returns a generic `Usd.Prim`.
To access specific features (like `GetSizeAttr`), you must wrap it in a Schema.
```python
# Generic
prim = stage.GetPrimAtPath("/World/MyCube")
# prim.GetSizeAttr() # ERROR! Generic prims don't have sizes.

# Typed
cube = UsdGeom.Cube(prim)
cube.GetSizeAttr() # Works!
```

## 5. Summary

Prims are the nodes of our world.
*   We create them with `.Define()`.
*   We modify them via **Schemas** (`UsdGeom.Cube`, `UsdGeom.Xform`).
*   We position them using `UsdGeom.XformCommonAPI`.

In the next lesson, we will learn how to build complex worlds by **Referencing** other USD files.