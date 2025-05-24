
"""
Script Name: blink_example.py
Author: Your Name
Date Created: 2025-04-23
Last Modified: 2025-04-23
Description:
    This script blinks an LED connected to GPIO 5 and detects button presses on GPIO 12.
    It uses the libgpiod library for GPIO control.

Revision History:
- 2025-04-23: Initial creation.
- 2025-04-23: Added button press detection functionality.
"""

import time
import gpiod
from gpiod.line import Direction, Value, Bias

LINE_OUTPUT = 5 # Physical Pin 29 (GPIO 5)
LINE_INPUT = 12 # Physical Pin 32 (GPIO 12)

with gpiod.request_lines(
    "/dev/gpiochip0",
    consumer="gpio-example",
    config={
        LINE_OUTPUT: gpiod.LineSettings(
            direction=Direction.OUTPUT,
            output_value=Value.ACTIVE
        ),
        LINE_INPUT: gpiod.LineSettings(
            direction=Direction.INPUT,
            bias=Bias.PULL_UP
        )
    }
) as request:
    while True:
        # Blink logic
        request.set_value(LINE_OUTPUT, Value.ACTIVE)
        time.sleep(1)
        request.set_value(LINE_OUTPUT, Value.INACTIVE)
        time.sleep(1)

        # Input test logic
        value = request.get_value(LINE_INPUT)
        if value == Value.INACTIVE:
            print("Button press detected")