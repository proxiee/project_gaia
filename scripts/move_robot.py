from omni.isaac.kit import SimulationApp

# --- Simulation setup ---
# This line is important, it sets up the simulation environment
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage

# --- Main Program ---
if __name__ == "__main__":
    # Open the scene we just created
    open_stage(usd_path="C:\\projects\\project_gaia\\assets\\kitchen_scene.usd") # IMPORTANT: Use the full, absolute path here

    # Create a World object to hold the simulation context
    world = World()
    # Reset the world to ensure a clean start
    world.reset()

    # --- Find the Robot and its joints ---
# We need to get a handle to the robot to control it.
# IMPORTANT: Before running, double-check in the Isaac Sim UI that your robot is actually named "panda" in the Stage panel.
# The prim path should be /World/panda
franka_prim_path = "/World/panda"
franka_robot = Articulation(franka_prim_path)

# Add the robot to the world's scene
world.scene.add(franka_robot)

# --- Define Target Positions ---
# You may need to adjust these values based on your scene layout
target_position = np.array([0.6, -0.2, 0.4])

# Add a small red cube to visualize our target
world.scene.add(
    VisualCuboid(
        prim_path="/World/target_cube",
        name="target_cube",
        position=target_position,
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([1.0, 0, 0]),
    )
)

# This is a crucial step that initializes the physics simulation
world.play()

# --- FIX IS HERE: Step the simulation a few times to ensure everything is loaded ---
# This gives the simulator time to register the robot from the loaded scene.
for _ in range(20):
    world.step()

# Now, we can safely get the gripper handle because we know the robot is loaded.
gripper_handle = franka_robot.get_joint_body("panda_hand")
if gripper_handle is None:
    raise Exception(f"Could not find gripper handle for robot at path: {franka_prim_path}")


# Main simulation loop
for i in range(500): # Run for 500 steps
    # Get the current position of the gripper
    gripper_position, _ = gripper_handle.get_world_pose()

    # A simple controller: calculate the direction to the target
    direction_to_target = target_position - gripper_position

    # Calculate the force to apply to move the gripper
    force = direction_to_target * 50.0

    # Apply the force to the gripper handle! This is what makes it move.
    gripper_handle.apply_force(force)

    # This steps the physics simulation forward by one frame
    world.step(render=True)

    # Stop if we are close to the target
    if np.linalg.norm(direction_to_target) < 0.02:
        print("Target reached!")
        break

# Stop the simulation
world.stop()
# Cleanup and close the simulation
simulation_app.close()