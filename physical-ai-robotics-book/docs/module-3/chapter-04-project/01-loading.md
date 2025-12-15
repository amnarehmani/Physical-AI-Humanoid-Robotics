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

## Introduction

In the graphical user interface of Isaac Sim, you drag and drop assets from the Content Browser into the viewport. But how do we do this programmatically? How do we ensure that when we press "Play" in our script, the physics engine knows exactly which objects to simulate?

This lesson introduces the **Core API**, specifically the `World` class and the `Robot` wrappers. These tools allow us to compose scenes dynamically, which is essential for "Domain Randomization" (varying the environment to make AI robust).

## Conceptual Understanding: The World Manager

At the heart of every Isaac Sim script is the `World` class. Think of `World` as the **Director** of a play. It does not act itself, but it manages everything that does.

The `World` singleton handles three critical systems:
1.  **The Timeline**: It controls the start, stop, pause, and step of the simulation time.
2.  **The Physics Scene**: It ensures a `PhysicsScene` Prim exists (defining gravity, solver steps, etc.).
3.  **The Scene Registry**: It keeps a list of all "active" objects (robots, sensors) that need to be reset or updated each cycle.

### Class Hierarchy

When we load a robot, we are often using a high-level wrapper that manages the underlying USD Prim.

```text
+---------------------+
|      USD Prim       |  (The raw data: meshes, materials, joints)
+---------------------+
          ^
          | wraps
          |
+---------------------+
|   Core.Articulation |  (Physics awareness: mass, stiffness, damping)
+---------------------+
          ^
          | inherits
          |
+---------------------+
|     Core.Robot      |  (Agent awareness: pose, initialization)
+---------------------+
          ^
          | specializes
          |
+---------------------+
|    WheeledRobot     |  (Kinematics awareness: wheel_dof_names)
+---------------------+
```

For a humanoid, we would typically inherit from `Robot` or `Articulation` directly, as `WheeledRobot` makes assumptions (like having a "chassis" and "wheels") that don't apply to legs.

## System Perspective: The Initialization Flow

Writing an Isaac Sim application follows a strict four-stage lifecycle. Violating this order is the most common source of bugs (e.g., "Articulation Handle not initialized").

```text
1. SETUP
   Initialize 'World'
   Set Physics Params (Gravity, dt)
         |
         v
2. COMPOSITION (Add)
   Add Ground Plane
   Add Robot (USD Reference)
   Add Obstacles
         |
         v
3. MATERIALIZATION (Reset)
   await world.reset()
   *CRITICAL*: This is when USD is parsed into PhysX C++ objects.
   Handles are acquired here.
         |
         v
4. EXECUTION (Loop)
   while running:
       Apply Actions
       Step Physics
       Read Sensors
```

## Implementation: The Warehouse Loader

Let's write the code to load NVIDIA's "Carter" robot into a basic environment.

### 1. Imports and Setup
We import `World` to manage the sim and `WheeledRobot` to manage our agent. `get_assets_root_path` finds the NVIDIA Nucleus server where the official assets live.

```python
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Initialize the World with a specific physics time step
# backend="numpy" allows us to get sensor data as numpy arrays directly
world = World(stage_units_in_meters=1.0, backend="numpy")

# Get the server path (e.g., omniverse://localhost/NVIDIA/...)
assets_root = get_assets_root_path()
if assets_root is None:
    print("Error: Could not find Nucleus server. Is Omniverse Launcher running?")
    exit()
```

### 2. Adding the Ground and Robot
We add assets to the `world.scene`. Note that `add()` does not spawn the object instantly; it registers it for the next reset.

```python
# Add a standard grey physics ground plane
world.scene.add_default_ground_plane()

# Define the path to the USD file for the robot
carter_usd_path = assets_root + "/Isaac/Robots/Carter/carter_v1.usd"

# Instantiate the wrapper
carter = world.scene.add(
    WheeledRobot(
        prim_path="/World/Carter",     # Where in the USD stage it will live
        name="my_carter",              # Unique internal name
        wheel_dof_names=["left_wheel", "right_wheel"], # Joint names in USD
        create_robot=True,             # Create the prim if it doesn't exist?
        usd_path=carter_usd_path,      # The asset to reference
        position=[0, 0, 0.5]           # Start slightly in the air (drop test)
    )
)
```

### 3. The Reset (Materialization)
This line is the most important. Before `reset()`, the robot is just a path string. After `reset()`, the physics engine has allocated memory for its rigid bodies and colliders.

```python
world.reset()
print("Robot loaded and physics initialized.")
```

### 4. The Simulation Loop
In a game engine, the loop happens automatically. In a scientific simulator, we control time. `world.step(render=True)` advances the physics simulation by one time-step (e.g., 1/60th of a second) and updates the visualization.

```python
# Assuming this is inside an Extension or standalone app wrapper
# For a standalone script, we usually loop checking the app status
while True:
    world.step(render=True)
    
    # Simple check to see if physics is running (user didn't press Pause)
    if world.is_playing():
        # This is where we will add control logic later
        pass
```

## Common Pitfalls

1.  **Forgetting `world.reset()`**: If you try to move the robot before resetting, you will get an error saying "Articulation view is not valid." The physics handle is only created *after* the first reset.
2.  **Incorrect Joint Names**: `wheel_dof_names` must match the *exact* names in the USD file. If you are using a custom humanoid, you must open the USD in the GUI and check the joint names in the Stage tree.
3.  **Z-Fighting**: If you spawn the robot at `z=0`, its wheels might be slightly inside the floor, causing it to explode into the air due to collision resolution. Always spawn slightly above ground (`z=0.5`).

## End-of-Lesson Checklist

- [ ] I can verify `get_assets_root_path()` returns a valid string.
- [ ] I have loaded the Carter robot into an empty world via script.
- [ ] I understand that `world.reset()` is required to initialize physics handles.
- [ ] I have verified the robot falls to the ground and settles (physics is working).
