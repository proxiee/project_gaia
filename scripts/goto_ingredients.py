# Phase 3: goto_ingredient.py
# This script combines Phase 1 (robot movement) and Phase 2 (perception)
# Goal: Find a specific ingredient and move the robot's hand to it.

import numpy as np
# We are using the reliable "omni.isaac.core" imports that we know work in your environment
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.core.articulations import Articulation

print("--- Running Phase 3: Intelligent Movement Script ---")

try:
    # Get the world instance
    world = World.instance()
    if world is None:
        raise Exception("World instance is None. Is the simulation running?")

    # 1. --- PERCEPTION ---
    # Define the object we want to move to.
    # You can change this to "/World/lemon_cube" to test another object!
    TARGET_INGREDIENT_PATH = "/World/tomato_cube"

    print(f"Attempting to find target: {TARGET_INGREDIENT_PATH}")

    # Get a handle to our target ingredient prim
    target_ingredient = VisualCuboid(
        prim_path=TARGET_INGREDIENT_PATH,
        name="target_ingredient_ref"
    )

    # Check if the ingredient was found
    if not target_ingredient.is_valid:
        raise Exception(f"Target ingredient not found at path: {TARGET_INGREDIENT_PATH}")

    # Get the 3D coordinates of our target. This is the goal for our robot.
    target_position, _ = target_ingredient.get_world_pose()
    print(f"Target found at position: {np.round(target_position, 3)}")


    # 2. --- ROBOT SETUP ---
    # Get a handle to the robot
    franka_robot = Articulation("/World/panda")
    # Get the gripper that we want to move
    gripper_handle = franka_robot.get_joint_body("panda_hand")

    if not franka_robot.is_valid or gripper_handle is None:
        raise Exception("Franka robot or its gripper handle could not be found.")

    # We need to explicitly add the robot to the scene for the articulated body API to work
    world.scene.add(franka_robot)


    # 3. --- MOVEMENT LOOP ---
    # Run the physics simulation
    world.play()
    
    # We will run the movement for a set number of steps
    for i in range(1000):
        world.step(render=True) # Step the simulation

        # Get the current position of the gripper
        gripper_position, _ = gripper_handle.get_world_pose()

        # Calculate the direction from the gripper to the target
        direction_to_target = target_position - gripper_position
        
        # If we are close enough, stop and celebrate
        if np.linalg.norm(direction_to_target) < 0.05:
            print(f"\nSUCCESS: Reached the {TARGET_INGREDIENT_PATH}!")
            break
        
        # Apply a force to the gripper to move it towards the target
        # This is our simple controller
        force = direction_to_target * 50.0
        gripper_handle.apply_force(force)

    world.stop() # Stop the physics
    print("--- Script Finished ---")


except Exception as e:
    print("An error occurred:")
    print(e)