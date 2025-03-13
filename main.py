#!/usr/bin/env python

import logging
import importlib.util
import sys

# Dynamically load the module from the file with a dash in its name.
module_name = "robot_arm_controller"
module_path = "./src/robot-arm-controller.py"

spec = importlib.util.spec_from_file_location(module_name, module_path)
robot_arm_controller = importlib.util.module_from_spec(spec)
sys.modules[module_name] = robot_arm_controller
spec.loader.exec_module(robot_arm_controller)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    robot_arm_controller.main()