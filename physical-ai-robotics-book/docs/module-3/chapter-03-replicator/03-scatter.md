---
id: m3-ch3-scatter
title: "Lesson 3: Advanced Scatter"
sidebar_label: "Lesson 3: Scatter"
description: "Scattering debris and objects naturally."
keywords:
  - replicator
  - scatter
  - physics
  - collision
---

# Lesson 3: Advanced Scatter

## The Floating Object Problem

If you just randomize `z` position between 0 and 1, objects might spawn inside the floor or floating in mid-air.
We want objects to spawn, fall, collide, and settle naturally.

## Physics-Based Randomization

Replicator can work with the physics engine.

```python
# 1. Spawn objects high up
rep.modify.pose(position=rep.distribution.uniform((-1, 5, -1), (1, 10, 1)))

# 2. Let physics run
# The 'rep.step()' command waits for physics to settle before taking the photo.
```

## Scatter2D

For creating cluttered floors (screws, leaves, trash), use `rep.randomizer.scatter_2d`.

```python
plane = rep.create.plane(scale=5)
screws = rep.create.from_usd("screw.usd", count=50)

with rep.trigger.on_frame():
    rep.randomizer.scatter_2d(
        screws, 
        surface=plane, 
        check_collisions=True
    )
```
This algorithm attempts to place the 50 screws on the plane without them overlapping.

## Lights and Textures

Don't forget to randomize the environment.

```python
# Randomize Lights
lights = rep.create.light(count=3)
with rep.trigger.on_frame():
    with lights:
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 3000))
        rep.modify.attribute("color", rep.distribution.uniform((0,0,0), (1,1,1)))
```

## End-of-Lesson Checklist

- [ ] I can use `scatter_2d` to populate a surface.
- [ ] I understand how collision checking prevents overlapping spawns.
- [ ] I have randomized lighting conditions.
- [ ] I have generated a dataset of a "Cluttered Floor."
