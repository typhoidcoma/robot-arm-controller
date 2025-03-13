"""
This script lets you control your robot using an Xbox controller’s analog sticks and right trigger.
The robot arm is controlled as follows:
- Left Stick Vertical:   Move forward/backward (affects X)
- Left Stick Horizontal: Move left/right (affects Y)
- Right Stick Vertical:  Move up/down (affects Z)
- Right Trigger:         Controls gripper continuously (0.0 = closed, 1.0 = open)
"""

import pygame
import requests
import time
import logging
from typing import Literal

# --- Configuration ---
BASE_URL: str = "http://127.0.0.1:80/"  # Replace with your actual base URL
STEP_SIZE: int = 2                      # Movement step multiplier (cm per unit analog input)
SLEEP_TIME: float = 0.05                # Loop sleep time (20 Hz)

# Global gripper state (float: 1.0 means fully open, 0.0 means fully closed)
open_state: float = 1.0

# Axis indices (adjust if necessary for your controller)
LEFT_STICK_HORIZONTAL_AXIS = 0
LEFT_STICK_VERTICAL_AXIS = 1
RIGHT_STICK_VERTICAL_AXIS = 3  # Change this if your controller uses a different index.
RIGHT_TRIGGER_AXIS = 5

# Ask user about relative position using numeric input.
def behind_or_front() -> Literal["Behind", "Facing"]:
    while True:
        inp = input("Enter 1 if you are behind your robot or 2 if you are facing your robot: ")
        if inp == "1":
            print("You chose 'Behind'")
            return "Behind"
        elif inp == "2":
            print("You chose 'Facing'")
            return "Facing"
        else:
            print("Invalid input. Please enter 1 for Behind or 2 for Facing.")

user_position: Literal["Behind", "Facing"] = behind_or_front()

# Deadzone threshold for analog sticks (to filter noise)
DEADZONE = 0.15

# Tolerance for detecting a significant change in the gripper value
GRIPPER_TOLERANCE = 0.05

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
            json={"x": 0, "y": 0, "z": 0, "rx": 1.5, "ry": 0, "rz": 0, "open": 1.0},
            timeout=5,
        )
        response.raise_for_status()
        logging.info("Robot initialized successfully")
    except requests.exceptions.RequestException as e:
        logging.error(f"Failed to initialize robot: {e}")
    time.sleep(1)  # Allow time for the robot to initialize

def control_robot():
    """Control the robot using Xbox controller analog stick and trigger inputs with /move/relative."""
    endpoint = f"{BASE_URL}move/relative"

    logging.info("Control the robot using the Xbox controller:")
    logging.info("  Left Stick Vertical:   Move forward/backward (affects X)")
    logging.info("  Left Stick Horizontal: Move left/right (affects Y)")
    logging.info("  Right Stick Vertical:  Move up/down (affects Z)")
    logging.info("  Right Trigger:         Controls gripper continuously (0.0 = closed, 1.0 = open)")
    logging.info("Press Ctrl+C to exit")

    # Initialize Pygame and the joystick module.
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        logging.error("No joystick detected. Please connect an Xbox controller and try again.")
        pygame.quit()
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    logging.info(f"Initialized controller: {joystick.get_name()}")

    global open_state
    previous_gripper_value = open_state  # used for detecting significant changes

    try:
        while True:
            # Pump the event queue to update joystick state.
            for event in pygame.event.get():
                pass

            # --- Read analog stick inputs ---
            left_horizontal = joystick.get_axis(LEFT_STICK_HORIZONTAL_AXIS)
            left_vertical = joystick.get_axis(LEFT_STICK_VERTICAL_AXIS)
            right_vertical = joystick.get_axis(RIGHT_STICK_VERTICAL_AXIS)

            # Apply deadzone filtering.
            left_horizontal = 0 if abs(left_horizontal) < DEADZONE else left_horizontal
            left_vertical = 0 if abs(left_vertical) < DEADZONE else left_vertical
            right_vertical = 0 if abs(right_vertical) < DEADZONE else right_vertical

            # --- Compute movement deltas ---
            # Mapping changes based on user position.
            if user_position == "Behind":
                delta_x = -left_vertical * STEP_SIZE
                delta_y = -left_horizontal * STEP_SIZE
            else:
                delta_x = left_vertical * STEP_SIZE
                delta_y = left_horizontal * STEP_SIZE

            # Right stick vertical controls Z (up/down). Pushing up (negative value) yields positive Z.
            delta_z = -right_vertical * STEP_SIZE

            # --- Read right trigger for gripper control ---
            raw_trigger = joystick.get_axis(RIGHT_TRIGGER_AXIS)
            # Normalize trigger value to range 0 (released) to 1 (fully pressed).
            normalized_trigger = (raw_trigger + 1) / 2
            # Compute gripper value: 1.0 (open) when not pressed, 0.0 (closed) when fully pressed.
            new_gripper_value = 1.0 - normalized_trigger

            # Check if the change is significant.
            send_gripper_command = abs(new_gripper_value - previous_gripper_value) > GRIPPER_TOLERANCE
            previous_gripper_value = new_gripper_value
            open_state = new_gripper_value

            # --- Send movement command if any movement is detected or gripper state changes significantly ---
            if abs(delta_x) > 0 or abs(delta_y) > 0 or abs(delta_z) > 0 or send_gripper_command:
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
                    logging.info(
                        f"Sent movement: x={delta_x:.2f}, y={delta_y:.2f}, z={delta_z:.2f}, open={open_state:.2f}"
                    )
                except requests.exceptions.RequestException as e:
                    logging.error(f"Failed to send movement command: {e}")

            time.sleep(SLEEP_TIME)
    except KeyboardInterrupt:
        logging.info("Exiting control loop...")
    finally:
        pygame.quit()

def main():
    logging.basicConfig(level=logging.INFO)
    init_robot()
    control_robot()

if __name__ == "__main__":
    main()
