from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import carb

def main():
    # 1. Initialize the Simulation World
    # physics_dt=1.0/60.0 ensures 60 physics steps per second
    world = World(stage_units_in_meters=1.0, backend="torch")
    world.scene.add_default_ground_plane()
    
    # 2. Load the Environment (USD Asset from Nucleus)
    # This URL points to a standard Isaac Sim warehouse asset
    warehouse_usd = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")

    # 3. Load the Humanoid Robot
    # In a real workflow, we convert URDF to USD using the Isaac URDF Importer extension.
    # Here we use a placeholder reference for demonstration.
    robot_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.2.1/Isaac/Robots/Carter/carter_v1.usd" # Placeholder for User's Robot
    add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")
    
    # 4. Initialize the Physics Context
    world.reset()
    carb.log_info("Simulation Started. Exploring the Digital Twin...")

    # 5. Simulation Loop
    # We step the physics engine forward frame-by-frame
    for i in range(1000):
        world.step(render=True)
        
    world.stop()

if __name__ == "__main__":
    main()
