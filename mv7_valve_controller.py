
#mv7_valve_controller.py ver 0.5.0

import gpiod
import time
import socket
from gpiod.line import Direction, Value, Bias

class MV7ValveController:
    def __init__(self, gpio_control, mcp, sock=None):
        self.gpio = gpio_control
        self.mcp = mcp
        self.sock = sock
        self.GPIO_DIRECTION = 27# LOW = CCW, HIGH = CW
        self.GPIO_MOTOR = 17# LOW = motor stop, HIGH = motor start
        self.GPIO_POSITION = 4# LOW = valve position reached

    def read_position(self):
        gpb_state = self.mcp.bus.read_byte_data(self.mcp.address, 0x13)# GPIOB register
        bit_1 = (gpb_state & 0b01000000) >> 6# GPB6
        bit_0 = (gpb_state & 0b10000000) >> 7# GPB7

        if bit_1 == 0 and bit_0 == 1:
            return "LOAD"
        elif bit_1 == 1 and bit_0 == 0:
            return "INJECT"
        elif bit_1 == 1 and bit_0 == 1:
            return "WASH"
        else:
            return "UNKNOWN"

    def get_direction_and_steps(self, current, target):
        if current == target:
            return None, 0
        transitions = {
            "LOAD": {"INJECT": ("CW", 1), "WASH": ("CW", 2)},
            "INJECT": {"LOAD": ("CCW", 1), "WASH": ("CW", 1)},
            "WASH": {"INJECT": ("CCW", 1), "LOAD": ("CCW", 2)}
        }
        return transitions.get(current, {}).get(target, (None, 0))

    def move_to_position(self, target_position):
        current_position = self.read_position()
        direction, steps = self.get_direction_and_steps(current_position, target_position)
        if direction is None:
            print(f"Valve already in {current_position} position")
            return True #Already in position

        self.gpio.set_value(self.GPIO_DIRECTION, Value.ACTIVE if direction == "CW" else Value.INACTIVE)
        print(f"Direction set to {direction}, moving {steps} step(s)")

        for step in range(steps):
            self.gpio.set_value(self.GPIO_MOTOR, Value.ACTIVE)
            print(f"Step {step + 1}: Motor activated")
            time.sleep(0.1)

            timeout = 0.5
            poll_interval = 0.01
            elapsed = 0
            while elapsed < timeout:
                if self.gpio.get_value(self.GPIO_POSITION) == Value.INACTIVE:
                    print("Position detected! Stopping motor.")
                    break
                time.sleep(poll_interval)
                elapsed += poll_interval
            else:
                print("Timeout: No position signal detected.")

            self.gpio.set_value(self.GPIO_MOTOR, Value.INACTIVE)
            time.sleep(0.5)
        self.gpio.set_value(self.GPIO_DIRECTION, Value.INACTIVE)
        new_position = self.read_position()
        print(f"New valve position = {new_position}")
        
        # Emit status to client.py and server
        if hasattr(self, 'sock') and self.sock:
            try:
                if new_position != target_position:
                    self.sock.sendall("Valve Malfunction".encode('utf-8'))
                else:
                    self.sock.sendall(f"VALVE_POSITION:{new_position}".encode('utf-8'))
            except Exception as e:
                print(f"Socket error sending valve status: {e}")

        return new_position == target_position

    def cleanup(self):
        print("Valve controller cleanup complete (no GPIO lines to release).")



