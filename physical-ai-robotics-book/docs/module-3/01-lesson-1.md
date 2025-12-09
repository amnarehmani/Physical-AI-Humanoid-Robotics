---
id: 01-lesson-1
title: "Photorealistic Simulation (Isaac Sim)"
sidebar_label: "1. Isaac Sim"
description: "Introduction to NVIDIA Isaac Sim and USD for photorealistic robotics simulation."
keywords:
  - isaac sim
  - nvidia
  - usd
  - simulation
  - photorealism
---

# Lesson 1: Photorealistic Simulation (Isaac Sim)

## 1.1 Why Isaac Sim?

Gazebo is a "Physics-First" simulator. It prioritizes dynamics over looks.
Isaac Sim is a "Photo-First" simulator built on **NVIDIA Omniverse**. It uses:
*   **RTX Rendering**: Ray-tracing for realistic light and shadows.
*   **USD (Universal Scene Description)**: A powerful file format for 3D worlds.
*   **Synthetic Data Generation**: Creating labelled datasets for training AI models.

## 1.2 Setting Up the Environment

Isaac Sim uses a "Nucleus" server to manage assets. We will load a pre-built warehouse environment.

### Python Scripting for Isaac Sim

Unlike Gazebo's XML launch files, Isaac Sim is controlled via Python.

```python title="code/module-3/isaac_sim/load_humanoid.py"
# This script loads a humanoid robot into an Isaac Sim warehouse
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

def main():
    # 1. Create the World
    world = World()
    world.scene.add_default_ground_plane()
    
    # 2. Load the Warehouse Asset (USD)
    warehouse_usd = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")

    # 3. Load the Robot
    # Note: You would replace this with your converted URDF->USD path
    robot_usd = "/path/to/your/robot.usd"
    add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")
    
    # 4. Run Simulation
    world.reset()
    print("Simulation Started. Press Ctrl+C to exit.")
    while True:
        world.step(render=True)

if __name__ == "__main__":
    main()
```

<h2>1.3 Exercise: The Warehouse</h2>

1.  Launch Isaac Sim.
2.  Open the **Script Editor**.
3.  Paste the code above (adjusting paths if necessary).
4.  **Run**. You should see your robot standing in a photorealistic warehouse.