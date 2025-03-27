#!/usr/bin/env python3

"""
This script lets you control your robot using an Xbox controller.
The robot arm is controlled using the following controls:
- Left Stick Up/Down:  Move forward/backward
- Right Stick Left/Right: Rotate clockwise/counter-clockwise (control rz)
- Right Trigger: Open/Close gripper
- Left Bumper: Rotate ry negative (down)
- Right Bumper: Rotate ry positive (up)
- D-pad Up: Rotate rx positive
- D-pad Down: Rotate rx negative
- B Button: Reinitialize the robot
- Y Button: Increase z (move up)
- A Button: Decrease z (move down)
"""

# Standard library imports
import requests  # For making HTTP requests to the robot's API
import time      # For adding delays between commands
import logging   # For logging information and errors
from typing import cast  # For type castingging
import pygame    # For interfacing with the Xbox controller
from typing import Literal  # Type annotations


# Configuration
BASE_URL: str = "http://127.0.0.1:80/"  # Robot's API endpoint

# Independent step sizes for different movement axes
X_STEP_SIZE: float = 3.0  # Forward/backward movement step in centimeters
Y_STEP_SIZE: float = 5.0  # Left/right movement step in centimeters (not currently used)
Z_STEP_SIZE: float = 1.0  # Up/down movement step in centimeters

# Independent rotation speeds for different rotation axes
RX_ROTATION_SPEED: float = 5.0  # Rotation speed around X axis (D-pad up/down)
RY_ROTATION_SPEED: float = 5.0  # Rotation speed around Y axis (bumpers)
RZ_ROTATION_SPEED: float = 3.0  # Rotation speed around Z axis (right stick)

SLEEP_TIME: float = 0.005  # Loop sleep time (20 Hz) - controls responsiveness
DEADZONE: float = 0.15  # Joystick deadzone to prevent drift when controller is at rest

# Global open state (initially 1 as set in init_robot)
# 1 = open, 0 = closed
open_state: Literal[0, 1] = 1

# Track absolute position of the robot
current_pos = {"x": 0, "y": 0, "z": 0, "rx": 1.5, "ry": 0, "rz": 0}


def behind_or_front() -> Literal["Behind", "Facing"]:
    """
    Returns the relative position of the user to the robot.
    This affects control orientation so movements are intuitive from the user's perspective.
    
    Returns:
        "Behind" or "Facing" based on user input
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


# Store user position to adjust control orientation
user_position: Literal["Behind", "Facing"] = behind_or_front()


def init_robot() -> None:
    """
    Initialize the robot by calling /move/init and setting an absolute starting position.
    
    This resets the robot to a known state and prepares it for manual control.
    The starting position has the arm extended with gripper open.
    """
    global current_pos
    endpoint_init = f"{BASE_URL}move/init"  # API endpoint for initialization
    endpoint_absolute = f"{BASE_URL}move/absolute"  # API endpoint for absolute positioning
    try:
        # First initialize the robot system
        response = requests.post(endpoint_init, json={}, timeout=5)
        response.raise_for_status()
        time.sleep(2)  # Wait for initialization to complete
        
        # Then set the robot to a known starting position
        initial_position = {"x": 0, "y": 0, "z": 0, "rx": 1.5, "ry": 0, "rz": 0, "open": 1}
        response = requests.post(
            endpoint_absolute,
            json=initial_position,
            timeout=5,
        )
        response.raise_for_status()
        
        # Update our position tracker
        current_pos["x"] = initial_position["x"]
        current_pos["y"] = initial_position["y"]
        current_pos["z"] = initial_position["z"]
        current_pos["rx"] = initial_position["rx"]
        current_pos["ry"] = initial_position["ry"]
        current_pos["rz"] = initial_position["rz"]
        
        logging.info("Robot initialized successfully")
    except requests.exceptions.RequestException as e:
        logging.error(f"Failed to initialize robot: {e}")
    time.sleep(1)  # Allow time for the robot to initialize


def apply_deadzone(value: float, deadzone: float) -> float:
    """
    Apply deadzone to controller input to prevent drift.
    
    Args:
        value: The raw joystick input value (-1.0 to 1.0)
        deadzone: The deadzone threshold (0.0 to 1.0)
        
    Returns:
        Adjusted value with deadzone applied and normalized
    """
    if abs(value) < deadzone:
        return 0.0  # Zero out small inputs to prevent drift
    # Normalize the value considering the deadzone
    # This ensures smooth control transition from deadzone to max
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - deadzone) / (1 - deadzone)


def control_robot():
    """
    Control the robot with keyboard inputs using /move/relative.
    
    This is the main control loop that:
    1. Reads controller inputs
    2. Calculates movement commands
    3. Sends commands to the robot
    4. Handles gripper open/close operations
    """
    global current_pos, open_state
    endpoint = f"{BASE_URL}move/relative"  # API endpoint for relative movement

    # Initialize pygame and joystick subsystems
    pygame.init()
    pygame.joystick.init()
    
    # Check if any joysticks/controllers are connected
    if pygame.joystick.get_count() == 0:
        logging.error("No joysticks found. Please connect an Xbox controller.")
        return
    
    # Initialize the first joystick found
    controller = pygame.joystick.Joystick(0)
    controller.init()
    logging.info(f"Connected to controller: {controller.get_name()}")

    # Display control instructions
    logging.info("Control the end effector using the Xbox controller:")
    logging.info("  Left Stick Up/Down:  Move forward/backward")
    logging.info("  Right Stick Left/Right: Rotate clockwise/counter-clockwise")
    logging.info("  Right Trigger: Open/Close gripper")
    logging.info("  Left Bumper: Rotate ry negative (down)")
    logging.info("  Right Bumper: Rotate ry positive (up)")
    logging.info("  D-pad Up: Rotate rx positive")
    logging.info("  D-pad Down: Rotate rx negative")
    logging.info("  B Button: Reinitialize the robot")
    logging.info("  Y Button: Increase z (move up)")
    logging.info("  A Button: Decrease z (move down)")
    logging.info("Press Ctrl+C to exit")

    # Track trigger state for toggling gripper (prevents continuous toggling)
    trigger_pressed = False
    b_button_pressed = False
    global open_state

    # Button mappings - may vary slightly between controllers
    RIGHT_BUMPER = 5  # RB - Right Bumper (used for ry positive rotation)
    LEFT_BUMPER = 4   # LB - Left Bumper (used for ry negative rotation)
    B_BUTTON = 1      # B Button (used for re-initializing the robot)
    Y_BUTTON = 3      # Y Button (used for increasing z)
    A_BUTTON = 0      # A Button (used for decreasing z)
    
    # Axis mappings - these may vary between different controller models
    LEFT_STICK_Y = 1   # Vertical axis of left stick (up/down)
    RIGHT_STICK_X = 2  # Horizontal axis of right stick (left/right)
    RIGHT_TRIGGER = 5  # Right trigger axis

    try:
        # Main control loop
        while True:
            # Process pygame events to keep the controller responsive
            pygame.event.pump()
            
            # Get joystick inputs with deadzone compensation
            # Invert Y axis as pygame has negative values for up (counterintuitive)
            left_stick_y = -apply_deadzone(controller.get_axis(LEFT_STICK_Y), DEADZONE)
            right_stick_x = apply_deadzone(controller.get_axis(RIGHT_STICK_X), DEADZONE)
            # Normalize trigger from [-1,1] range to [0,1] for easier threshold checking
            right_trigger = (controller.get_axis(RIGHT_TRIGGER) + 1) / 2
            
            # Check bumper buttons for ry rotation control
            left_bumper_pressed = controller.get_button(LEFT_BUMPER)
            right_bumper_pressed = controller.get_button(RIGHT_BUMPER)
            
            # Check Y and A buttons for z-axis control
            y_button_pressed = controller.get_button(Y_BUTTON)
            a_button_pressed = controller.get_button(A_BUTTON)
            
            # Get D-pad (hat) values - returns (x,y) tuple where y=1 is up, y=-1 is down
            hat_x, hat_y = controller.get_hat(0)
            dpad_up_pressed = (hat_y == 1)
            dpad_down_pressed = (hat_y == -1)
            
            # Adjust for user position (Behind/Facing) to maintain intuitive controls
            if user_position == "Facing":
                left_stick_y = -left_stick_y
                right_stick_x = -right_stick_x
            
            # Calculate movement based on stick positions with independent step sizes
            delta_x = left_stick_y * X_STEP_SIZE       # Forward/backward with left stick
            delta_y = 0                                # No left/right control
            delta_z = 0                                # Initialize z movement
            
            # Apply z-axis control from Y and A buttons
            if y_button_pressed:
                delta_z = Z_STEP_SIZE     # Move up with Y button
            elif a_button_pressed:
                delta_z = -Z_STEP_SIZE    # Move down with A button
                
            # Apply rotation with independent rotation speeds
            delta_rz = right_stick_x * RZ_ROTATION_SPEED  # Rotation with right stick left/right
            
            # Calculate ry rotation based on bumper buttons
            delta_ry = 0
            if left_bumper_pressed:
                delta_ry = -RY_ROTATION_SPEED  # Negative ry rotation with left bumper
            elif right_bumper_pressed:
                delta_ry = RY_ROTATION_SPEED   # Positive ry rotation with right bumper
                
            # Calculate rx rotation based on D-pad buttons
            delta_rx = 0
            if dpad_up_pressed:
                delta_rx = RX_ROTATION_SPEED   # Positive rx rotation with D-pad up
            elif dpad_down_pressed:
                delta_rx = -RX_ROTATION_SPEED  # Negative rx rotation with D-pad down
            
            # Check if B button was just pressed to reinitialize robot
            b_button = controller.get_button(B_BUTTON)
            if b_button and not b_button_pressed:
                logging.info("B button pressed - reinitializing robot...")
                init_robot()
                logging.info("Robot reinitialized")
            b_button_pressed = b_button
            
            # Check right trigger for gripper control (toggle on press, not hold)
            if right_trigger > 0.7 and not trigger_pressed:
                # Toggle gripper state between open (1) and closed (0)
                open_state = 0 if open_state == 1 else 1
                data = {
                    "x": 0, "y": 0, "z": 0, 
                    "rx": 0, "ry": 0, "rz": 0, 
                    "open": open_state
                }
                try:
                    # Send gripper command to the robot
                    response = requests.post(endpoint, json=data, timeout=1)
                    response.raise_for_status()
                    logging.info(f"Toggled gripper to {'open' if open_state == 1 else 'closed'}")
                except requests.exceptions.RequestException as e:
                    logging.error(f"Failed to toggle gripper: {e}")
            
            # Update trigger state for next iteration
            trigger_pressed = right_trigger > 0.7
            
            # Send movement command if joystick is being used (any significant movement) or rotation is happening
            if abs(delta_x) > 0.01 or abs(delta_y) > 0.01 or abs(delta_z) > 0.01 or abs(delta_rx) > 0.01 or abs(delta_rz) > 0.01 or abs(delta_ry) > 0.01:
                # Update our tracked position
                current_pos["x"] += delta_x
                current_pos["y"] += delta_y
                current_pos["z"] += delta_z
                current_pos["rx"] += delta_rx
                current_pos["rz"] += delta_rz
                current_pos["ry"] += delta_ry
                
                data = {
                    "x": delta_x,
                    "y": delta_y,
                    "z": delta_z,
                    "rx": delta_rx,  # Include rx rotation from D-pad
                    "ry": delta_ry,  # Apply rotation around Y axis using bumpers
                    "rz": delta_rz,  # Apply rotation around Z axis
                    "open": open_state,  # Maintain current gripper state
                }
                try:
                    # Send movement command to the robot
                    response = requests.post(endpoint, json=data, timeout=1)
                    response.raise_for_status()
                    if delta_rx != 0 or delta_rz != 0 or delta_ry != 0:
                        logging.info(f"Rotating: rx={delta_rx:.2f}, ry={delta_ry:.2f}, rz={delta_rz:.2f}, Current position: x={current_pos['x']:.1f}, y={current_pos['y']:.1f}, z={current_pos['z']:.1f}")
                    else:
                        logging.info(f"Movement: x={delta_x:.2f}, y={delta_y:.2f}, z={delta_z:.2f}, Current position: x={current_pos['x']:.1f}, y={current_pos['y']:.1f}, z={current_pos['z']:.1f}")
                except requests.exceptions.RequestException as e:
                    logging.error(f"Request failed: {e}")
                    # Revert our position tracking on failure
                    current_pos["x"] -= delta_x
                    current_pos["y"] -= delta_y
                    current_pos["z"] -= delta_z
                    current_pos["rx"] -= delta_rx
                    current_pos["rz"] -= delta_rz
                    current_pos["ry"] -= delta_ry
            
            # Brief sleep to control the loop rate
            time.sleep(SLEEP_TIME)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        logging.info("Exiting...")
    finally:
        # Clean up pygame resources on exit
        pygame.quit()


# Add a debugging function to identify controller axes
def print_controller_info():
    """
    Print information about the connected controller to help with debugging.
    This can be called to identify which axis corresponds to which stick movement.
    """
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No controllers found!")
        return
        
    controller = pygame.joystick.Joystick(0)
    controller.init()
    
    print(f"Controller Name: {controller.get_name()}")
    print(f"Number of Axes: {controller.get_numaxes()}")
    print(f"Number of Buttons: {controller.get_numbuttons()}")
    
    print("\nMove the RIGHT STICK left and right to verify axis mapping...")
    print("Press buttons to see their numbers...")
    print("Press Ctrl+C to exit")
    
    try:
        while True:
            pygame.event.pump()
            # Print all axis values to help identify which is the right stick horizontal
            print("\nAxis Values:")
            for i in range(controller.get_numaxes()):
                print(f"Axis {i}: {controller.get_axis(i):.3f}")
            
            # Print all button states
            print("\nButton States:")
            for i in range(controller.get_numbuttons()):
                if controller.get_button(i):
                    print(f"Button {i} pressed")
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()


def main():
    """
    Main entry point for the program.
    Sets up logging, initializes the robot, and starts the control loop.
    """
    # Configure logging to show timestamps and log levels
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    # Uncomment the line below to debug controller axis mappings
    # print_controller_info()
    
    init_robot()  # Initialize the robot before starting control
    control_robot()  # Start the main control loop


# Script entry point
if __name__ == "__main__":
    main()