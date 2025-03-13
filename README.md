# Xbox Controller Robot Arm Controller

This project provides a Python script to control a robot arm using an Xbox controller. The script maps analog stick inputs to movement commands and uses the right trigger to control the gripper's open/close state. It sends these commands via HTTP POST requests to a robot endpoint.

## Features

- **Analog Stick Control:**  
  - **Left Stick:**  
    - Vertical movement controls forward/backward motion (affecting the X-axis).  
    - Horizontal movement controls left/right motion (affecting the Y-axis).
  - **Right Stick:**  
    - Vertical movement controls up/down motion (affecting the Z-axis).
- **Gripper Control:**  
  - The right trigger is used to control the gripper: when pressed beyond a threshold, the gripper closes; otherwise, it remains open.
- **User Position Configuration:**  
  - At startup, the user is prompted to specify whether they are "Behind" or "Facing" the robot. This input adjusts how the left stick's inputs are mapped to movement directions.
- **HTTP Communication:**  
  - Movement commands and gripper state changes are sent to the robot using HTTP POST requests.

## Prerequisites

- Python 3.6+
- [pygame](https://www.pygame.org/) (for reading Xbox controller inputs)
- [requests](https://docs.python-requests.org/) (for HTTP requests)

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/yourusername/robot-arm-controller.git
   cd robot-arm-controller
