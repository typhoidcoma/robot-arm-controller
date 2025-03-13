"""
This script lets you control your robot using an Xbox controller’s analog sticks and right trigger.
The robot arm is controlled as follows:
- Left Stick Vertical:   Move forward/backward (affects X)
- Left Stick Horizontal: Move left/right (affects Y)
- Right Stick Vertical:  Move up/down (affects Z)
- Right Trigger:         Control gripper (pressed → closed, released → open)
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
open_state: Literal[0, 1] = 1            # Global open state (1: open, 0: closed)

# Ask user about relative position (affects mapping of left stick axes)
def behind_or_front() -> Literal["Behind", "Facing"]:
    while True:
        inp = input("Type 'Behind' if you are behind your robot or 'Facing' if you are facing your robot: ")
        if inp in ["Behind", "Facing"]:
            print(f"You chose '{inp}'")
            return inp  # type: Literal["Behind", "Facing"]
        else:
            print("You must choose between 'Behind' or 'Facing'")

user_position: Literal["Behind", "Facing"] = behind_or_front()

# Deadzone threshold for analog sticks (to filter noise)
DEADZONE = 0.15

# Threshold for the right trigger (after normalization) to consider it "pressed"
TRIGGER_THRESHOLD = 0.5

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

def control_robot():
    """Control the robot using Xbox controller analog stick and trigger inputs with /move/relative."""
    endpoint = f"{BASE_URL}move/relative"

    logging.info("Control the robot using the Xbox controller:")
    logging.info("  Left Stick Vertical:   Move forward/backward (affects X)")
    logging.info("  Left Stick Horizontal: Move left/right (affects Y)")
    logging.info("  Right Stick Vertical:  Move up/down (affects Z)")
    logging.info("  Right Trigger:         Close gripper when pressed")
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

    try:
        while True:
            # Pump the event queue to update joystick state.
            for event in pygame.event.get():
                pass

            # --- Read analog stick inputs ---
            # Left stick: axis 0 (horizontal), axis 1 (vertical)
            left_horizontal = joystick.get_axis(0)
            left_vertical = joystick.get_axis(1)
            # Right stick vertical: adjust axis index as needed (commonly axis 3 or 4)
            right_vertical = joystick.get_axis(4)

            # Apply deadzone filtering.
            left_horizontal = 0 if abs(left_horizontal) < DEADZONE else left_horizontal
            left_vertical = 0 if abs(left_vertical) < DEADZONE else left_vertical
            right_vertical = 0 if abs(right_vertical) < DEADZONE else right_vertical

            # --- Compute movement deltas ---
            # For user "Behind":
            #   - Pushing left stick up (negative value) moves the robot forward (positive X)
            #   - Pushing left stick right (positive value) moves the robot right (negative Y)
            if user_position == "Behind":
                delta_x = -left_vertical * STEP_SIZE
                delta_y = -left_horizontal * STEP_SIZE
            else:
                delta_x = left_vertical * STEP_SIZE
                delta_y = left_horizontal * STEP_SIZE

            # Right stick vertical controls Z (up/down). Pushing up (negative value) → positive Z.
            delta_z = -right_vertical * STEP_SIZE

            # --- Read right trigger for gripper control ---
            # Assume the right trigger is on axis 5.
            # Many controllers report trigger values from -1 (released) to 1 (fully pressed).
            raw_trigger = joystick.get_axis(5)
            # Normalize to 0 (released) to 1 (fully pressed).
            trigger_value = (raw_trigger + 1) / 2

            # Determine gripper state: if the trigger is pressed beyond the threshold, close gripper.
            global open_state
            open_state = 0 if trigger_value > TRIGGER_THRESHOLD else 1

            # --- Send movement command if any movement is detected ---
            if abs(delta_x) > 0 or abs(delta_y) > 0 or abs(delta_z) > 0 or trigger_value > TRIGGER_THRESHOLD:
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
                    logging.info(f"Sent movement: x={delta_x:.2f}, y={delta_y:.2f}, z={delta_z:.2f}, open={open_state}")
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