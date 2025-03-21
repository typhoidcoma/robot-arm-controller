#!/usr/bin/env python3

"""
This script lets you control your robot using an Xbox controller.
The robot arm is controlled using the following controls:
- Left Stick Up/Down:  Move forward/backward
- Right Stick Left/Right: Move left/right
- Right Bumper (RB): Move up (increase Z)
- Left Bumper (LB): Move down (decrease Z)
- Right Trigger: Open/Close gripper
"""

import requests
import time
import logging
from typing import cast
import pygame
from typing import Dict, Literal, Tuple


# Configuration
BASE_URL: str = "http://192.168.1.8:80/"
STEP_SIZE: int = 5.0  # Movement step in centimeters
SLEEP_TIME: float = 0.005  # Loop sleep time (20 Hz)
DEADZONE: float = 0.15  # Joystick deadzone to prevent drift

# Global open state (initially 1 as set in init_robot)
open_state: Literal[0, 1] = 1


def behind_or_front() -> Literal["Behind", "Facing"]:
    """
    Returns the relative position of the user to the robot.
    """
    while True:
        inp = input(
            "Type 'Behind' if you are behind your robot or 'Facing' if you are facing your robot: "
        )
        if inp in ["Behind", "Facing"]:
            print(f"You chose '{inp}'")
            inp = cast(Literal["Behind", "Facing"], inp)
            return inp
        else:
            print("You must choose between 'Behind' or 'Facing'")


user_position: Literal["Behind", "Facing"] = behind_or_front()


def init_robot() -> None:
    """Initialize the robot by calling /move/init and setting an absolute starting position."""
    endpoint_init = f"{BASE_URL}move/init"
    endpoint_absolute = f"{BASE_URL}move/absolute"
    try:
        response = requests.post(endpoint_init, json={}, timeout=5)
        response.raise_for_status()
        time.sleep(2)
        response = requests.post(
            endpoint_absolute,
            json={"x": 0, "y": 0, "z": 0, "rx": 1.5, "ry": 0, "rz": 0, "open": 1},
            timeout=5,
        )
        response.raise_for_status()
        logging.info("Robot initialized successfully")
    except requests.exceptions.RequestException as e:
        logging.error(f"Failed to initialize robot: {e}")
    time.sleep(1)  # Allow time for the robot to initialize


def apply_deadzone(value: float, deadzone: float) -> float:
    """Apply deadzone to controller input to prevent drift."""
    if abs(value) < deadzone:
        return 0.0
    # Normalize the value considering the deadzone
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone) / (1 - deadzone)


def control_robot():
    """Control the robot with Xbox controller inputs using /move/relative."""
    endpoint = f"{BASE_URL}move/relative"

    # Initialize pygame and joystick
    pygame.init()
    pygame.joystick.init()
    
    # Check if any joysticks/controllers are connected
    if pygame.joystick.get_count() == 0:
        logging.error("No joysticks found. Please connect an Xbox controller.")
        return
    
    # Initialize the first joystick
    controller = pygame.joystick.Joystick(0)
    controller.init()
    logging.info(f"Connected to controller: {controller.get_name()}")

    logging.info("Control the end effector using the Xbox controller:")
    logging.info("  Left Stick Up/Down:  Move forward/backward")
    logging.info("  Right Stick Left/Right: Move left/right")
    logging.info("  Right Bumper (RB): Move up (increase Z)")
    logging.info("  Left Bumper (LB): Move down (decrease Z)")
    logging.info("  Right Trigger: Open/Close gripper")
    logging.info("Press Ctrl+C to exit")

    # Track trigger state for toggling
    trigger_pressed = False
    global open_state

    # Button mappings - may vary slightly between controllers
    RIGHT_BUMPER = 5  # RB - Right Bumper
    LEFT_BUMPER = 4   # LB - Left Bumper

    try:
        while True:
            # Process pygame events
            pygame.event.pump()
            
            # Get joystick inputs with deadzone compensation
            # Invert Y axis as pygame has negative values for up
            left_stick_y = -apply_deadzone(controller.get_axis(1), DEADZONE)
            right_stick_x = apply_deadzone(controller.get_axis(2), DEADZONE)
            right_trigger = (controller.get_axis(5) + 1) / 2  # Normalize from [-1,1] to [0,1]
            
            # Get bumper states for Z-axis movement
            right_bumper_pressed = controller.get_button(RIGHT_BUMPER)  # A key equivalent
            left_bumper_pressed = controller.get_button(LEFT_BUMPER)    # D key equivalent
            
            # Adjust for user position (Behind/Facing)
            if user_position == "Facing":
                left_stick_y = -left_stick_y
                right_stick_x = -right_stick_x
            
            # Calculate movement based on stick positions
            delta_x = left_stick_y * STEP_SIZE  # Forward/backward with left stick
            delta_y = right_stick_x * STEP_SIZE  # Left/right with right stick
            
            # Calculate Z-axis movement based on bumper buttons
            delta_z = 0
            if right_bumper_pressed:
                delta_z += STEP_SIZE  # Move up (increase Z) with right bumper
            if left_bumper_pressed:
                delta_z -= STEP_SIZE  # Move down (decrease Z) with left bumper
            
            # Check right trigger for gripper control
            if right_trigger > 0.7 and not trigger_pressed:
                # Toggle gripper state
                open_state = 0 if open_state == 1 else 1
                data = {
                    "x": 0, "y": 0, "z": 0, 
                    "rx": 0, "ry": 0, "rz": 0, 
                    "open": open_state
                }
                try:
                    response = requests.post(f"{BASE_URL}move/relative", json=data, timeout=1)
                    response.raise_for_status()
                    logging.info(f"Toggled gripper to {'open' if open_state == 1 else 'closed'}")
                except requests.exceptions.RequestException as e:
                    logging.error(f"Failed to toggle gripper: {e}")
            
            trigger_pressed = right_trigger > 0.7
            
            # Send movement command if joystick is being used
            if abs(delta_x) > 0.01 or abs(delta_y) > 0.01 or abs(delta_z) > 0.01:
                data = {
                    "x": delta_x,
                    "y": delta_y,
                    "z": delta_z,
                    "rx": 0,
                    "ry": 0,
                    "rz": 0,
                    "open": open_state,
                }
                try:
                    response = requests.post(endpoint, json=data, timeout=1)
                    response.raise_for_status()
                    logging.info(f"Sent movement: x={delta_x:.2f}, y={delta_y:.2f}, z={delta_z:.2f}")
                except requests.exceptions.RequestException as e:
                    logging.error(f"Request failed: {e}")
            
            time.sleep(SLEEP_TIME)
    except KeyboardInterrupt:
        logging.info("Exiting...")
    finally:
        pygame.quit()


def main():
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    init_robot()
    control_robot()


if __name__ == "__main__":
    main()