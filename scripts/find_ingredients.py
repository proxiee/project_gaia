# Phase 2: find_ingredients.py
# This script loads the scene and identifies the 3D coordinates of our named objects.

from isaacsim import SimulationApp

# Standard simulation setup
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.simulation_context import SimulationContext
from isaacsim.core.objects import VisualCuboid # We use this class to wrap our existing cubes
from isaacsim.core.utils.stage import open_stage_async

# --- Main Program ---
if __name__ == "__main__":
    # Create the SimulationContext
    simulation_context = SimulationContext()

    # Load our kitchen scene
    # IMPORTANT: Remember to use the full, absolute path to your scene file
    stage_path = "C:/projects/project_gaia/assets/kitchen_scene.usd"
    open_stage_async(stage_path).wait()
    
    # We don't need to run the physics for this, but we need to load the context
    simulation_context.initialize_physics()
    simulation_context.play() # Use play to ensure stage is fully loaded
    simulation_context.step() # Step once to update the scene graph

    print("Searching for ingredients in the scene...")

    # --- This is the core logic for perception ---
    
    # A list of the prim names we want to find (these must match the names in your scene)
    ingredient_prim_paths = [
        "/World/tomato_cube",
        "/World/lemon_cube",
        "/World/knife_block"
    ]
    
    # A dictionary to store the results
    ingredient_positions = {}

    for prim_path in ingredient_prim_paths:
        # We create a VisualCuboid object using the prim_path of the object ALREADY in the stage.
        # This gives us a Python handle to the object.
        # We give it a unique name for the program, like "tomato_ref"
        ingredient_object = VisualCuboid(
            prim_path=prim_path, 
            name=prim_path.split("/")[-1] + "_ref" # e.g., "tomato_cube_ref"
        )
        
        # Now, get its 3D world position and orientation
        position, orientation = ingredient_object.get_world_pose()
        
        # Store the position in our dictionary
        ingredient_positions[prim_path] = position
        print(f"Found {prim_path} at position: {position}")

    print("\n--- Perception Complete ---")
    print("Final Ingredient Map:")
    for name, pos in ingredient_positions.items():
        # Print with nice formatting, rounding the numbers
        print(f"-> {name}: [ {np.round(pos[0], 3)}, {np.round(pos[1], 3)}, {np.round(pos[2], 3)} ]")
    print("-------------------------")

    # Stop and shutdown
    simulation_context.stop()
    simulation_app.close()