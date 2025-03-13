#!/usr/bin/env python

import logging
from src.robot_arm_controller import main

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()