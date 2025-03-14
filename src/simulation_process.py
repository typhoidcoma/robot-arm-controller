import os
import sys
import time
import pygame
import mujoco
import logging
from multiprocessing import Queue
import numpy as np

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s: %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger("simulation")

def simulation_loop(state_queue: Queue):

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
    logger.info("Number of joystick hats: %d", num_hats)
    logger.info("Initial axis values: %s", [joystick.get_axis(i) for i in range(num_axes)])
    logger.info("Initial hat values: %s", [joystick.get_hat(i) for i in range(num_hats)])

    MODEL_XML = "robots\\trs_so_arm100\\scene.xml"
    if not os.path.exists(MODEL_XML):
        logger.error("Model file '%s' not found!", MODEL_XML)
        sys.exit()
    model = mujoco.MjModel.from_xml_path(MODEL_XML)
    data = mujoco.MjData(model)
    
    rotation_scale    = 0.05   # For Rotation (left stick horizontal, axis 0)
    pitch_scale       = 0.05   # For Pitch (left stick vertical, axis 1)
    elbow_scale       = 0.1   # For Elbow (right stick vertical, axis 4)
    wrist_roll_scale  = 0.8   # For Wrist_Roll (right stick horizontal, axis 3)
    wrist_pitch_scale = 0.1   # For Wrist_Pitch (D-Pad vertical, hat 0)
    jaw_scale         = 0.2   # For Jaw (right trigger, axis 5) – inverted
    bumper_scale = 0.05  

    clamp_min = -3.14
    clamp_max = 3.14

    target_ctrl = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    data.ctrl[:] = target_ctrl

    dt = model.opt.timestep
    logger.info("Simulation timestep: %.5f", dt)

    iteration = 0
    while True:
        # Process Pygame events.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                logger.info("Joystick quit event received. Terminating simulation.")
                return

        rotation_input = joystick.get_axis(0)  
        pitch_input    = joystick.get_axis(1) 
        elbow_input = joystick.get_axis(3)  
        wrist_pitch_input = joystick.get_button(11)          
        jaw_input = joystick.get_axis(5)   

        wrist_pitch_input = 0.0
        if joystick.get_button(0):
            wrist_pitch_input += 1.0
        if joystick.get_button(1):
            wrist_pitch_input -= 1.0

        wrist_roll_input = 0.0  
        if joystick.get_button(4):
            wrist_roll_input -= bumper_scale  
        if joystick.get_button(5):
            wrist_roll_input += bumper_scale 

        # ----- Accumulate: Update target positions by adding incremental changes.
        target_ctrl[0] += rotation_scale    * rotation_input * dt   # Rotation
        target_ctrl[1] += pitch_scale       * pitch_input    * dt   # Pitch
        target_ctrl[2] += elbow_scale       * elbow_input    * dt   # Elbow
        target_ctrl[3] += wrist_pitch_scale * wrist_pitch_input * dt   # Wrist_Pitch
        target_ctrl[4] += wrist_roll_scale  * wrist_roll_input  * dt   # Wrist_Roll
        target_ctrl[5] -= jaw_scale         * jaw_input      * dt   # Jaw (inverted)

        # Clamp each joint's target value between -3.14 and 3.14.
        for i in range(len(target_ctrl)):
            target_ctrl[i] = np.clip(target_ctrl[i], clamp_min, clamp_max)
        
        data.ctrl[:] = target_ctrl[:]
        mujoco.mj_step(model, data)
        iteration += 1

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
        # No sleep for maximum update rate.
        
if __name__ == "__main__":
    from multiprocessing import Queue
    queue = Queue()
    simulation_loop(queue)
