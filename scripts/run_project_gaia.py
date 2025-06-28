# The Final Standalone Script for Phase 3
# This script launches Isaac Sim, finds a target, and moves the robot to it.
# It is designed to be run from the command line with python.bat ONLY.

# We use the modern 'isaacsim' imports because the standalone method supports them.
from isaacsim import SimulationApp

# Standard simulation setup
CONFIG = {"headless": False}
simulation_app = SimulationApp(CONFIG)

import numpy as np
from isaacsim.core.simulation_context import SimulationContext
from isaacsim.core.objects import VisualCuboid
# The modern class for a single robot is SingleArticulation
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import open_stage_async
import traceback

def run_simulation():
    """ The main function that orchestrates the entire simulation sequence. """
    try:
        # Get the simulation context
        simulation_context = SimulationContext.instance()

        # Load our kitchen scene
        stage_path = "C:/projects/project_gaia/assets/kitchen_scene.usd"
        open_stage_async(stage_path).wait()

        # IMPORTANT: Initialize physics and play the simulation BEFORE accessing objects
        simulation_context.initialize_physics()
        simulation_context.play()
        # Step once to process all the physics handles and ensure objects are ready
        simulation_context.step()

        print("--- Perception Stage ---")
        # Find the target ingredient
        target_ingredient = VisualCuboid(
            prim_path="/World/tomato_cube",
            name="target_ingredient_ref"
        )
        
        print("--- Robot Setup Stage ---")
        # Use the correct name we found in the Stage panel
        robot_prim_path = "/World/Franka" 
        
        # Use the modern SingleArticulation class
        franka_robot = SingleArticulation(
            prim_path=robot_prim_path,
            name="franka_robot_view" # Use a unique name for the view
        )
        # Initialize the robot after the stage is playing
        franka_robot.initialize()
        # Use the modern .get_body() method
        gripper_handle = franka_robot.get_body("panda_hand")

        if not gripper_handle:
            raise Exception(f"Could not get gripper handle for robot at {robot_prim_path}")

        # Final reset before starting the main loop
        simulation_context.reset()
        franka_robot.reinitialize() # Re-initialize the robot after reset

        # Get the target position AFTER resetting
        target_position, _ = target_ingredient.get_world_pose()
        print(f"Target locked at: {np.round(target_position, 3)}")


        print("--- Movement Stage ---")
        # Main application loop
        while simulation_app.is_running():
            simulation_context.step()

            if simulation_context.is_playing():
                gripper_position, _ = gripper_handle.get_world_pose()
                
                if np.linalg.norm(gripper_position) < 0.01:
                    continue

                direction_to_target = target_position - gripper_position
                
                if np.linalg.norm(direction_to_target) < 0.05:
                    print("\nSUCCESS: Target Reached!")
                    break # Exit the inner while loop
                
                force = direction_to_target * 50.0
                gripper_handle.apply_force(force)
        
        # This break is outside the inner loop to exit the outer one after success
        # A bit redundant with the app close, but clean.
        # This part of the original code was flawed. Let's simplify.
        # The success case will now just let the app close naturally.


    except Exception:
        print("An error occurred:")
        traceback.print_exc()
    finally:
        # Always shut down the simulation
        print("--- Shutting down ---")
        simulation_app.close()

# The script's entry point
if __name__ == "__main__":
    run_simulation()