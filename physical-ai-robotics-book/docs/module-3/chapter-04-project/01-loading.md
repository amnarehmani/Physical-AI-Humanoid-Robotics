---
id: m3-ch4-loading
title: "Lesson 1: Loading the Scene"
sidebar_label: "Lesson 1: Scene Setup"
description: "Loading assets via World API."
keywords:
  - isaac sim
  - world
  - scene
  - carter
---

# Lesson 1: Loading the Scene

## The World Class

The `World` class manages the simulation timeline and physics context.

```python
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.nucleus import get_assets_root_path

# 1. Initialize
world = World()
assets_root = get_assets_root_path()

# 2. Add Ground
world.scene.add_default_ground_plane()

# 3. Add Robot (NVIDIA Carter)
carter_usd = assets_root + "/Isaac/Robots/Carter/carter_v1.usd"
carter = world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",
        name="my_carter",
        wheel_dof_names=["left_wheel", "right_wheel"],
        create_robot=True,
        usd_path=carter_usd
    )
)

# 4. Reset
world.reset()
```

## Running the Simulation Step

Unlike Replicator, we must manually step the physics.

```python
while simulation_app.is_running():
    world.step(render=True)
    if world.is_playing():
        # Control logic goes here
        pass
```

## Why WheeledRobot?

The `WheeledRobot` class is a wrapper. It handles the low-level USD joints and provides a high-level function: `apply_wheel_actions(ArticulationAction)`. This abstracts away the complexity of torque control.

## End-of-Lesson Checklist

- [ ] I can verify `get_assets_root_path()` is working.
- [ ] I have loaded the Carter robot into an empty world.
- [ ] I understand the render loop (`world.step`).
