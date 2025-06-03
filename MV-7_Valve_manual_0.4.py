

#!/usr/bin/env python3
# MV-7 Valve Test Script_0.4 for Raspberry Pi 5 using gpiod v2.3

import gpiod
import time
from datetime import timedelta
from gpiod.line import Direction, Value, Bias
from smbus2 import SMBus

# Initialize MCP23017
bus = SMBus(1)
mcp_address = 0x21

# MCP23017 Register Addresses

# Force GPB6 and GPB7 as inputs, leave others unchanged
IODIRB = 0x01
GPPUB = 0x0D

# Set GPB6 and GPB7 as inputs (bits 6 and 7 = 1)
bus.write_byte_data(mcp_address, IODIRB, 0b11000000)

# Enable pull-ups on GPB6 and GPB7
bus.write_byte_data(mcp_address, GPPUB, 0b11000000)

# Confirm
confirmed_iodirb = bus.read_byte_data(mcp_address, IODIRB)
print(f"DEBUG: Confirmed IODIRB = {confirmed_iodirb:08b}")


# Define GPIO pins
GPIO_DIRECTION = 27# LOW = CW, HIGH = CCW
GPIO_MOTOR = 17# LOW = motor ON, HIGH = motor OFF
GPIO_POSITION = 4# Falling edge = valve position reached

# Request output lines (GPIO27 and GPIO17)
output_configs = {
    GPIO_DIRECTION: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},
    GPIO_MOTOR: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'}
}
output_request = {}

for line_num, config in output_configs.items():
    initial_value = Value.ACTIVE if config['initial_state'] == 'HIGH' else Value.INACTIVE
    line_settings = gpiod.LineSettings(
        direction=config['direction'],
        output_value=initial_value
    )
    output_request[line_num] = gpiod.request_lines(
        "/dev/gpiochip0",
        consumer="mv7_test",
        config={line_num: line_settings}
    )

# Request GPIO4 for falling edge detection
position_settings = gpiod.LineSettings(
    direction=Direction.INPUT,
    bias=Bias.PULL_UP,
    
)
position_request = gpiod.request_lines(
    "/dev/gpiochip0",
    consumer="mv7_test",
    config={GPIO_POSITION: position_settings}
)

# Function to read valve position
def ReadValvePosition():
    gpb_state = bus.read_byte_data(mcp_address, 0x13)# GPIOB register
    bit_1 = (gpb_state & 0b01000000) >> 6# GPB6
    bit_0 = (gpb_state & 0b10000000) >> 7# GPB7

    # Debug output
    print(f"DEBUG: Raw GPIOB = {gpb_state:08b} (bit_1 = {bit_1}, bit_0 = {bit_0})")


    if bit_1 == 0 and bit_0 == 1:
        return "LOAD"
    elif bit_1 == 1 and bit_0 == 0:
        return "INJECT"
    elif bit_1 == 1 and bit_0 == 1:
        return "WASH"
    else:
        return "UNKNOWN"

# Determine direction and steps
def GetDirectionAndSteps(current, target):
    if current == target:
        return None, 0
    transitions = {
        "LOAD": {"INJECT": ("CW", 1), "WASH": ("CW", 2)},
        "INJECT": {"LOAD": ("CCW", 1), "WASH": ("CW", 1)},
        "WASH": {"INJECT": ("CCW", 1), "LOAD": ("CCW", 2)}
    }
    return transitions.get(current, {}).get(target, (None, 0))

try:
    # Print current valve position before motor activation
    current_position = ReadValvePosition()
    print(f"Current valve position = {current_position}")
    target_position = input("Enter target valve position (LOAD, INJECT, WASH): ").strip().upper()

    direction, steps = GetDirectionAndSteps(current_position, target_position)
    if direction is None:
        print(f"Valve already in {current_position} position or invalid target.")
    else:
        output_request[GPIO_DIRECTION].set_value(
            GPIO_DIRECTION,
            gpiod.line.Value.INACTIVE if direction == "CW" else gpiod.line.Value.ACTIVE
        )
        print(f"Direction set to {direction}, moving {steps} step(s)")

        for step in range(steps):
            output_request[GPIO_MOTOR].set_value(GPIO_MOTOR, Value.INACTIVE)
            print(f"Step {step + 1}: Motor activated")
            time.sleep(0.1)

            timeout = 0.5
            poll_interval = 0.01
            elapsed = 0
            while elapsed < timeout:
                if position_request.get_value(GPIO_POSITION) == Value.INACTIVE:
                    print("Position detected! Stopping motor.")
                    break
                time.sleep(poll_interval)
                elapsed += poll_interval
            else:
                print("Timeout: No position signal detected.")

            output_request[GPIO_MOTOR].set_value(GPIO_MOTOR, Value.ACTIVE)
            time.sleep(0.5)

        new_position = ReadValvePosition()
        print(f"New valve position = {new_position}")


except KeyboardInterrupt:
    print("Interrupted by user. Stopping motor.")
    output_request[GPIO_MOTOR].set_value(GPIO_MOTOR, Value.ACTIVE)

finally:
    # Release GPIO lines
    for line in output_request.values():
        line.release()
    position_request.release()
    print("GPIO lines released. Test complete.")

