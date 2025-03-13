# Xbox Controller Robot Arm Controller

This project provides a Python script that lets you control a robot arm using an Xbox controller’s analog sticks and right trigger. The script sends movement commands via HTTP POST requests to a defined robot endpoint.

## Features

- **Analog Stick Control:**  
  - **Left Stick:**  
    - **Vertical Axis:** Controls forward/backward motion (affects the X-axis).
    - **Horizontal Axis:** Controls left/right motion (affects the Y-axis).
  - **Right Stick:**  
    - **Vertical Axis:** Controls up/down motion (affects the Z-axis).
- **Gripper Control:**  
  - **Right Trigger:** Adjusts the gripper position continuously based on how hard the trigger is pressed. A fully pressed trigger closes the gripper (0.0), and a released trigger keeps it open (1.0).
- **User Position Configuration:**  
  - At startup, you are prompted to enter **1** if you are behind the robot or **2** if you are facing the robot. This setting determines how the left stick inputs are mapped to the X and Y axes.

## Prerequisites

- Python 3.6+
- [pygame](https://www.pygame.org/) (for reading Xbox controller inputs)
- [requests](https://docs.python-requests.org/) (for HTTP requests)

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/yourusername/robot-arm-controller.git
   cd robot-arm-controller
