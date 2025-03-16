# Description: This script is used to simulate the robot arm using the MuJoCo physics engine.
#
# The simulation_loop function initializes the Pygame joystick and loads the MuJoCo model. It then reads the joystick inputs and updates the target control values based on the input. The control values are clamped between -3.14 and 3.14 and applied to the model. The state of the model is periodically sent to the state_queue for visualization.

import os
import sys
import pygame
import mujoco
import logging
from multiprocessing import Queue
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("simulation")


def apply_dead_zone(value, threshold=0.1):
    """Return 0 if the absolute value of input is below the threshold, otherwise return the value."""
    return 0 if abs(value) < threshold else value


def simulation_loop(state_queue: Queue):
    # Initialize Pygame and joystick modules for controller input
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        logger.error("No joystick connected. Please connect an Xbox controller.")
        sys.exit()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    logger.info("Joystick initialized: %s", joystick.get_name())
    num_axes = joystick.get_numaxes()
    num_hats = joystick.get_numhats()
    logger.info("Number of joystick axes: %d", num_axes)
    # Log initial joystick axis and hat values for debugging
    logger.info("Initial axis values: %s", [joystick.get_axis(i) for i in range(num_axes)])
    logger.info("Initial hat values: %s", [joystick.get_hat(i) for i in range(num_hats)])
    
    # Load the MuJoCo model for simulation
    MODEL_XML = "robots\\trs_so_arm100\\scene.xml"
    if not os.path.exists(MODEL_XML):
        logger.error("Model file '%s' not found!", MODEL_XML)
        sys.exit()
    model = mujoco.MjModel.from_xml_path(MODEL_XML)
    data = mujoco.MjData(model)

    # Define control scales for each joint movement
    rotation_scale = 0.05  # Left stick horizontal controls rotation
    pitch_scale = 0.05     # Left stick vertical controls pitch
    elbow_scale = 0.1      # Right stick vertical controls elbow movement
    wrist_roll_scale = 0.8 # Right stick horizontal controls wrist roll
    wrist_pitch_scale = 0.1 # A button and B button control wrist pitch
    jaw_scale = 0.2        # Right trigger controls jaw (inverted)
    bumper_scale = 0.05    # Bumpers for adjusting wrist roll

    # Set clamping limits for joint control values (in radians)
    clamp_min = -3.14
    clamp_max = 3.14

    # Initialize the target control values for each joint
    target_ctrl = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    data.ctrl[:] = target_ctrl

    dt = model.opt.timestep
    logger.info("Simulation timestep: %.5f", dt)

    iteration = 0
    while True:
        # Process Pygame events (e.g. detecting window close events)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                logger.info("Joystick quit event received. Terminating simulation.")
                return

        # Read joystick inputs with dead zone filtering
        rotation_input = apply_dead_zone(joystick.get_axis(0))
        pitch_input = apply_dead_zone(joystick.get_axis(1))
        elbow_input = apply_dead_zone(joystick.get_axis(3))
        jaw_input = joystick.get_axis(5)  # Pre-read jaw input (will be inverted later)

        # Compute wrist pitch input based on button presses (using buttons 0 and 1)
        wrist_pitch_input = 0.0
        if joystick.get_button(0):
            wrist_pitch_input += 1.0
        if joystick.get_button(1):
            wrist_pitch_input -= 1.0

        # Compute wrist roll input based on bumper presses (buttons 4 and 5)
        wrist_roll_input = 0.0
        if joystick.get_button(4):
            wrist_roll_input -= bumper_scale
        if joystick.get_button(5):
            wrist_roll_input += bumper_scale

        # Incrementally update each control value based on joystick input and simulation timestep
        target_ctrl[0] += rotation_scale * rotation_input * dt  # Rotation
        target_ctrl[1] += pitch_scale * pitch_input * dt  # Pitch
        target_ctrl[2] += elbow_scale * elbow_input * dt  # Elbow
        target_ctrl[3] += wrist_pitch_scale * wrist_pitch_input * dt  # Wrist_Pitch
        target_ctrl[4] += wrist_roll_scale * wrist_roll_input * dt  # Wrist_Roll
        target_ctrl[5] -= jaw_scale * jaw_input * dt  # Jaw (inversion applied)

        # Clamp each joint's control value within the defined limits
        for i in range(len(target_ctrl)):
            target_ctrl[i] = np.clip(target_ctrl[i], clamp_min, clamp_max)

        # Apply control values to the model and advance the simulation
        data.ctrl[:] = target_ctrl[:]
        mujoco.mj_step(model, data)
        iteration += 1

        # Every 100 iterations, send the simulation state for visualization/update
        if iteration % 100 == 0:
            state = {
                'time': data.time,
                'qpos': data.qpos[:].tolist(),
                'ctrl': data.ctrl[:].tolist()
            }
            try:
                state_queue.put(state, block=False)
            except Exception as e:
                logger.error("Failed to put state on queue: %s", e)
        # No sleep for maximum update rate


if __name__ == "__main__":
    from multiprocessing import Queue
    queue = Queue()
    simulation_loop(queue)
