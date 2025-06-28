# The Definitive Standalone Script for Phase 3
# This script uses the STANDALONE method with the OLD (but necessary) API imports.

from omni.isaac.kit import SimulationApp

# Standard simulation setup
CONFIG = {"headless": False}
simulation_app = SimulationApp(CONFIG)

# All other imports must happen after the SimulationApp is created.
import numpy as np
import traceback
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import open_stage

def run_simulation():
    """ The main function that orchestrates the entire simulation sequence. """
    try:
        # We create a new world for this fresh simulation
        world = World()
        
        # Load the scene
        # Replace this with your full path if it's different
        stage_path = "C:/projects/project_gaia/assets/kitchen_scene.usd"
        open_stage(stage_path)

        # We must reset the world AFTER opening the stage
        world.reset()
        
        # Perform the simulation warm-up to ensure all objects are loaded
        print("Performing simulation warm-up...")
        for _ in range(60):
            world.step(render=False)
        print("...warm-up complete. World is ready.")

        # --- PERCEPTION STAGE ---
        TARGET_INGREDIENT_PATH = "/World/tomato_cube"
        print(f"Finding target: {TARGET_INGREDIENT_PATH}")
        target_ingredient = VisualCuboid(prim_path=TARGET_INGREDIENT_PATH)
        target_position, _ = target_ingredient.get_world_pose()
        print(f"Target found at position: {np.round(target_position, 3)}")

        # --- ROBOT SETUP STAGE ---
        robot_prim_path = "/World/Franka"
        print(f"Finding robot: {robot_prim_path}")
        franka_robot = Articulation(robot_prim_path)
        world.scene.add(franka_robot)
        gripper_handle = franka_robot.get_body("panda_hand")
        print("Robot handle acquired.")
        
        # --- MOVEMENT STAGE ---
        print("Starting movement loop...")
        while simulation_app.is_running():
            world.step(render=True)
            
            gripper_position, _ = gripper_handle.get_world_pose()
            if np.linalg.norm(gripper_position) < 0.01:
                continue

            direction_to_target = target_position - gripper_position
            if np.linalg.norm(direction_to_target) < 0.05:
                print("\nSUCCESS: Reached the target!")
                break
            
            force = direction_to_target * 50.0
            gripper_handle.apply_force(force)

    except Exception:
        print("\n--- AN ERROR OCCURRED ---")
        traceback.print_exc()
    finally:
        print("--- Shutting down ---")
        simulation_app.close()

# The script's entry point
if __name__ == "__main__":
    run_simulation()