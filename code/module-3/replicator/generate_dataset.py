import omni.replicator.core as rep

def generate_synthetic_data():
    # 1. Create the Environment
    # We create a simple plane and a light
    floor = rep.create.plane(scale=5, visible=True)
    light = rep.create.light(intensity=500, light_type="distant")

    # 2. Create the Assets
    # In a real scenario, this would load a USD file. Here we use a cube.
    # We attach semantic data for the annotator.
    target_object = rep.create.cube(semantics=[('class', 'target_box')], count=10)

    # 3. Create the Camera
    camera = rep.create.camera(position=(0, 10, 0), look_at=(0,0,0))
    render_product = rep.create.render_product(camera, (1024, 1024))

    # 4. Define Randomization Logic
    with rep.trigger.on_frame(num_frames=50):
        # Scatter the cubes on the floor
        rep.randomizer.scatter_2d(
            target_object,
            surface=floor,
            check_collisions=True,
            rotation=rep.distribution.uniform((0,0,0), (0,360,0))
        )
        
        # Randomize Light Color
        with light:
            rep.modify.attribute("color", rep.distribution.uniform((0.5, 0.5, 0.5), (1, 1, 1)))

    # 5. Define Writers (Output)
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="_output_dataset",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True
    )
    writer.attach([render_product])

    # 6. Run
    rep.orchestrator.run()

if __name__ == "__main__":
    generate_synthetic_data()
