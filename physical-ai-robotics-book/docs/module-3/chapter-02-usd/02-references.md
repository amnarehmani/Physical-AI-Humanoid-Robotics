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

## The "Import" Trap

In traditional CAD, you "Import" a file. It copies the data into your scene. If the original file changes, your scene is outdated.
In USD, we use **References**. It points to the file. If the original updates, your scene updates instantly.

## Adding a Reference

```python
from omni.isaac.core.utils.prims import define_prim

# Create an empty container
prim = define_prim("/World/Robot1", "Xform")

# Reference a USD file into it
prim.GetReferences().AddReference("path/to/robot.usd")
```

## Payloads: The Lazy Loader

A **Payload** is a "weak" Reference. You can choose *not* to load it.
Imagine a city scene with 100 buildings. You only need to work on Building A.
If used Payloads, you can "Unload" the other 99 buildings. They disappear from memory but stay in the scene hierarchy. This allows massive scenes to run on a single GPU.

## Overrides

You can reference a standard "Red Car" and then, in your scene, change its color to Blue. This is an **Override**.
It does not change the original file. It writes a small "opinion" in your current layer: "For this instance of Red Car, color = Blue."

## End-of-Lesson Checklist

- [ ] I understand the difference between Copying and Referencing.
- [ ] I can add a Reference to a Prim via Python.
- [ ] I know why Payloads are useful for large scenes.
- [ ] I have successfully overridden a property of a referenced asset.
