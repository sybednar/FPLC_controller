
#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
FPLC_client_0.4.4 (FPLC_controller) - added class GradientFlowrateController method logic and signaling
only handles pumpB 
Functions:
1) ADS1115 ADC data acquisition
2) Communicate with FPLC_server_GUI
3) Threading
4) GPIO communication
5) MCP23017 GPIO expander control
6) Fraction Collector control
7) Error detection and handling
8) PumpA wash control via MCP23017 GPA0
9) added valve control signal reception from FPLC_server

'''

import socket
import threading
import time
import Adafruit_ADS1x15
import gpiod
import json
from gpiod.line import Direction, Value, Bias
from smbus2 import SMBus
from mv7_valve_controller import MV7ValveController

# ---------------- GPIO Control ----------------
class GPIOControl:
    HIGH = Value.ACTIVE
    LOW = Value.INACTIVE

    def __init__(self, chip_name, gpio_configs):
        self.chip_name = chip_name
        self.gpio_configs = gpio_configs
        self.request = {}
        self.setup_gpio()

    def setup_gpio(self):
        for line_num, config in self.gpio_configs.items():
            initial_value = self.HIGH if config.get('initial_state') == 'HIGH' else self.LOW
            line_settings = gpiod.LineSettings(
                direction=config['direction'],
                output_value=initial_value if config['direction'] == Direction.OUTPUT else self.LOW,
                bias=config.get('bias', Bias.DISABLED)
            )
            self.request[line_num] = gpiod.request_lines(
                f"/dev/{self.chip_name}",
                consumer="gpio_control",
                config={line_num: line_settings}
            )

    def set_value(self, line_num, value):
        try:
            self.request[line_num].set_value(line_num, value)
        except PermissionError as e:
            print(f"Permission error: {e}")

    def get_value(self, line_num):
        return self.request[line_num].get_value(line_num)

    def monitor_input(self, line_num):
        value = self.get_value(line_num)
        return "HIGH" if value == self.HIGH else "LOW"

    def clear_gpio(self):
        for line_num in self.request:
            if line_num != 17: # Skip GPIO17 to avoid activating the valve motor
                self.set_value(line_num, self.LOW)

# ---------------- MCP23017 Control ----------------
class MCP23017:
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
        self.IODIRA = 0x00# I/O direction register for port A
        self.IODIRB = 0x01# I/O direction register for port B
        self.OLATA = 0x14# Output latch register for port A
        self.OLATB = 0x15# Output latch register for port B
        self.GPPUA = 0x0C# Pull-up resistor config for port A
        self.GPPUB = 0x0D# Pull-up resistor config for port B

        
        # For pump A Set GPA0, GPA1, GPA2 as outputs, GPA7 as inputs
        self.bus.write_byte_data(self.address, self.IODIRA, 0b10000000)
        self.bus.write_byte_data(self.address, self.GPPUA, 0b10000000)     
        self.set_pin_high('A', 0) #initialize GPA0 to HIGH (PumpA inactive wash)
        self.set_pin_high('A', 2) #initialize GPA2 to HIGH (PumpA internal speed control)
        
        # For pump B Set GPA0, GPB1, GPB2 as outputs, GPB3 as input
        self.bus.write_byte_data(self.address, self.IODIRB, 0b11001000)
        self.bus.write_byte_data(self.address, self.GPPUB, 0b11001000)      
        self.set_pin_high('B', 0) #initialize GPB0 to HIGH (PumpB inactive wash)
        self.set_pin_high('B', 2) #initialize GPB2 to HIGH (PumpB internal speed control)


    def set_pin_low(self, port, pin):
        if port == 'A':
            current_value = self.bus.read_byte_data(self.address, self.OLATA)
            new_value = current_value & ~(1 << pin)
            self.bus.write_byte_data(self.address, self.OLATA, new_value)
        elif port == 'B':
            current_value = self.bus.read_byte_data(self.address, self.OLATB)
            new_value = current_value & ~(1 << pin)
            self.bus.write_byte_data(self.address, self.OLATB, new_value)

    def set_pin_high(self, port, pin):
        if port == 'A':
            current_value = self.bus.read_byte_data(self.address, self.OLATA)
            new_value = current_value | (1 << pin)
            self.bus.write_byte_data(self.address, self.OLATA, new_value)
        elif port == 'B':
            current_value = self.bus.read_byte_data(self.address, self.OLATB)
            new_value = current_value | (1 << pin)
            self.bus.write_byte_data(self.address, self.OLATB, new_value)
            
    def read_pin(self, port, pin):
        if port == 'A':
            gpio_state = self.bus.read_byte_data(self.address, 0x12)# GPIOA register
        elif port == 'B':
            gpio_state = self.bus.read_byte_data(self.address, 0x13)# GPIOB register
        else:
            raise ValueError("Invalid port. Use 'A' or 'B'.")
        return (gpio_state >> pin) & 0x01
    
    def clear_all_outputs(self):
        # Set all output pins on both ports to LOW
        self.bus.write_byte_data(self.address, self.OLATA, 0x00)
        self.bus.write_byte_data(self.address, self.OLATB, 0x00)
        print("MCP23017: All outputs cleared (set to LOW)")


# ---------------- Pump Solvent Exchange Control ----------------
class SolventExchangePumps:
    def __init__(self, mcp, sock, valve_controller):
        self.mcp = mcp
        self.sock = sock
        self.valve_controller = valve_controller

        # Define pump configurations
        self.pump_configs = {
            "A": {
                "port": 'A',
                "pin_wash": 0,# GPA0 controls pumpA wash start
                "pin_status": 7, # GPA7 monitors pumpA wash status
                "pin_run": 1# GPA1 controls pumpA run state
            },
            "B": {
                "port": 'B',
                "pin_wash": 0,# GPB0 controls pumpB wash start
                "pin_status": 3, # GPB3 monitors pumpB wash status
                "pin_run": 1# GPB1 controls pumpB run state
            }
        }

    def start_wash(self, pump_ids):
        print(f"Starting solvent exchange for pumps: {pump_ids}")
        self.valve_controller.move_to_position("WASH")

        threads = []
        completion_flags = {pid: threading.Event() for pid in pump_ids}

        def monitor_pump(pump_id):
            config = self.pump_configs[pump_id]
            port = config["port"]
            pin_wash = config["pin_wash"]
            pin_status = config["pin_status"]
            pin_run = config["pin_run"]

            # Start wash
            self.mcp.set_pin_high(port, pin_run)# Run mode
            self.mcp.set_pin_low(port, pin_wash)# Activate wash
            print(f"Pump{pump_id} wash started")

            # Wait for wash to complete
            while self.mcp.read_pin(port, pin_wash) == 0:
                if self.mcp.read_pin(port, pin_status) == 1:
                    self.mcp.set_pin_high(port, pin_wash)# Deactivate wash
                    self.mcp.set_pin_low(port, pin_run)# Standby
                    print(f"Pump{pump_id} wash complete")
                    try:
                        self.sock.sendall(f"PUMP_{pump_id}_WASH_COMPLETED".encode('utf-8'))
                    except socket.error as e:
                        print(f"Socket error: {e}")
                    completion_flags[pump_id].set()
                    break
                time.sleep(0.5)

        # Start threads for each pump
        for pid in pump_ids:
            t = threading.Thread(target=monitor_pump, args=(pid,), daemon=True)
            threads.append(t)
            t.start()

        # Wait for all pumps to complete
        for pid in pump_ids:
            completion_flags[pid].wait()

        # Move valve to LOAD after all washes complete
        self.valve_controller.move_to_position("LOAD")
        print("All washes complete. Valve moved to LOAD.")




# ---------------- PumpA Flowrate Controller ----------------
class PumpAFlowrateController:
    def __init__(self, gpio_control, gpio_pinA, sock, mcp, data_acquisition, fraction_collector):
        self.gpio_control = gpio_control
        self.gpio_pinA = gpio_pinA #GPIO13 for PumpA
        self.mcp = mcp
        self.flowrate = 0.0
        self.thread = None
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.pump_runtime = 0.0
        self.volume_output = 0.0
        self.PumpA_volume_value = 0.0
        self.pause_event.set()
        self.pause_start_time = None
        self.total_pause_duration = 0
        self.last_sent_volume = -0.1
        self.sock = sock
        self.data_acquisition = data_acquisition
        self.fraction_collector = fraction_collector
        self.error_monitor_thread = threading.Thread(target=self.monitor_pump_error, daemon=True)

    def set_flowrate(self, flowrate):
        self.flowrate = flowrate
        
    def set_volume_value(self, volume_value):
        self.PumpA_volume_value = volume_value

    def calculate_period(self):
        frequency = self.flowrate * 18.315
        frequency = max(0, min(frequency, 153))
        return 1.0 / frequency if frequency > 0 else None

    def start(self):
        if self.thread is None or not self.thread.is_alive():
            
            print("Starting PumpA flowrate controller thread")

            self.stop_event.clear()
            self.thread = threading.Thread(target=self.run)
            self.thread.start()
            if not self.error_monitor_thread.is_alive():
                self.error_monitor_thread = threading.Thread(target=self.monitor_pump_error, daemon=True)
                self.error_monitor_thread.start()

    def run(self): 
        start_time = time.time()
        while not self.stop_event.is_set():
            self.pause_event.wait()
            period = self.calculate_period()
            if period is None:
                self.gpio_control.set_value(self.gpio_pinA, GPIOControl.LOW)
                time.sleep(0.1)
                continue
            half_period = period / 2.0
            self.gpio_control.set_value(self.gpio_pinA, GPIOControl.HIGH)
            time.sleep(half_period)
            self.gpio_control.set_value(self.gpio_pinA, GPIOControl.LOW)
            time.sleep(half_period)
            #print(f"Toggling GPIO{self.gpio_pinA} at {1/period:.2f} Hz")
            
            # Calculate runtime and volume output
            self.pump_runtime = time.time() - start_time - self.total_pause_duration
            self.volume_output = (self.flowrate/60) * self.pump_runtime
            print(f"Volume Delivered {self.volume_output} ml")
            
            #Send volume update every 1 mL restore line if to not show if data acquisition is NOT running
            current_volume_decile = round(self.volume_output*10) / 10.0
            if current_volume_decile > self.last_sent_volume:
                #if not (self.data_acquisition and self.data_acquisition.thread and self.data_acquisition.thread.is_alive()):
                try:
                    self.sock.sendall(f"PumpA_running {self.volume_output:.2f} ml".encode('utf-8'))
                    self.last_sent_volume = current_volume_decile
                except socket.error as e:
                    print(f"Socket send error (PumpA_running): {e}")

            # Check if volume output has reached the target
            if self.volume_output >= self.PumpA_volume_value:
                print(f"Volume output {self.volume_output:.2f} ml has reached the target {self.PumpA_volume_value:.2f} ml")
                self.mcp.set_pin_low('A', 1)# GPA1 LOW
                self.mcp.set_pin_high('A', 2)# GPA2 HIGH
                self.gpio_control.set_value(self.gpio_pinA, GPIOControl.LOW)# GPIO13 LOW
                
                #Notify server
                try:
                    self.sock.sendall("PumpA_stopped".encode('utf-8'))
                except socket.error as e:
                    print(f"Socket send error (PumpA_stopped): {e}")

                #Stop acquisition and fraction collector if running
                if self.data_acquisition and self.data_acquisition.thread and self.data_acquisition.thread.is_alive():
                    self.data_acquisition.stop()
                if self.fraction_collector and self.fraction_collector.thread and self.fraction_collector.thread.is_alive():
                    self.fraction_collector.stop()
                    self.fraction_collector.stop_fraction_collector()
                try:
                    self.sock.sendall("STOP_SAVE_ACQUISITION".encode('utf-8'))
                except socket.error as e:
                    print(f"Socket send error (STOP_SAVE_ACQUISITION): {e}")                
                
                self.stop_event.set()
                self.stop()

    def monitor_pump_error(self):
        previous_state = self.gpio_control.monitor_input(24)
        while not self.stop_event.is_set():
            current_state = self.gpio_control.monitor_input(24)
            if previous_state == "LOW" and current_state == "HIGH":
                print("PumpA error detected (GPIO24 HIGH)")
                self.pause()
                self.data_acquisition.pause()
                self.fraction_collector.pause()
                self.fraction_collector.pause_fraction_collector()
                self.send_pump_a_error_notification()

            elif previous_state == "HIGH" and current_state == "LOW":
                print("PumpA error cleared (GPIO24 LOW)")
                self.send_pump_a_error_cleared_notification()
                self.resume()
                self.data_acquisition.resume()
                self.fraction_collector.resume()
                self.fraction_collector.resume_fraction_collector()
                
            previous_state = current_state
            time.sleep(0.5)

    def send_pump_a_error_notification(self):
        try:
            self.sock.sendall("PumpA error".encode('utf-8'))
            print("Error notification sent to server")
        except socket.error as e:
            print(f"Socket send error: {e}")

    def send_pump_a_error_cleared_notification(self):
        try:
            self.sock.sendall("PumpA Error has been cleared".encode('utf-8'))
            print("Error cleared notification sent to server")
        except socket.error as e:
            print(f"Socket send error: {e}")

    def pause(self):
        print("PumpA paused")
        self.pause_start_time = time.time()
        self.pause_event.clear()

    def resume(self):
        if self.pause_start_time is not None:
            pause_duration = time.time() - self.pause_start_time
            self.total_pause_duration += pause_duration
            self.pause_start_time = None
        print("PumpA resumed")
        self.pause_event.set()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            if threading.current_thread() != self.thread:
                self.thread.join()
        self.gpio_control.set_value(self.gpio_pinA, GPIOControl.LOW)
        self.thread = None
        # Reset volume tracking for next run
        self.volume_output = 0.0
        self.last_sent_volume = -0.1


# ---------------- Gradient Flowrate Controller ----------------



class GradientFlowrateController:
    def __init__(self, gpio_control, gpio_pinA, gpio_pinB, sock, mcp, data_acquisition, fraction_collector):
        self.gpio_control = gpio_control
        self.gpio_pinA = gpio_pinA
        self.gpio_pinB = gpio_pinB
        self.sock = sock
        self.mcp = mcp

        self.flowrate = 0.0
        self.total_volume = 0.0
        self.volume_output = 0.0
        self.pumpB_min_percent = 0.0
        self.pumpB_max_percent = 100.0
        self.min_toggle_freq = 1.0

        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.pause_event.set()
        
        self.PumpA_volume_value = 0.0
        self.pause_start_time = None
        self.total_pause_duration = 0

        self.freq_A = 0.0
        self.freq_B = 0.0
        self.last_sent_volume = -0.1

        self.thread = None
        self.thread_A = None
        self.thread_B = None
        
        self.data_acquisition = data_acquisition
        self.fraction_collector = fraction_collector
        self.error_monitor_thread = threading.Thread(target=self.monitor_pump_error, daemon=True)

    def set_gradient_profile(self, min_percent, max_percent):
        self.pumpB_min_percent = min_percent
        self.pumpB_max_percent = max_percent

    def set_flowrate(self, flowrate):
        self.flowrate = flowrate

    def set_volume_value(self, volume_value):
        self.total_volume = volume_value

    def start(self):
        if self.thread is None or not self.thread.is_alive():
            self.stop_event.clear()
            self.thread = threading.Thread(target=self.run)
            self.thread.start()            
            if not self.error_monitor_thread.is_alive():
                self.error_monitor_thread = threading.Thread(target=self.monitor_pump_error, daemon=True)
                self.error_monitor_thread.start()
         
    def run(self):
        print("[Gradient] run() started")
        start_time = time.time()
        total_runtime = (self.total_volume / self.flowrate) * 60# seconds           
            
        def toggle_gpio(pin, get_freq, label):
            while not self.stop_event.is_set():
                self.pause_event.wait()
                freq = get_freq()
                print(f"[{label}] Toggling at {freq:.2f} Hz")
                if freq >= self.min_toggle_freq:
                    period = 1.0 / freq
                    self.gpio_control.set_value(pin, GPIOControl.HIGH)
                    time.sleep(period / 2)
                    self.gpio_control.set_value(pin, GPIOControl.LOW)
                    time.sleep(period / 2)
                else:
                    self.gpio_control.set_value(pin, GPIOControl.LOW)
                    time.sleep(0.1)

            print(f"[{label}] Exiting toggle thread.")

        self.thread_A = threading.Thread(target=toggle_gpio, args=(self.gpio_pinA, lambda: self.freq_A, "PumpA"))
        self.thread_B = threading.Thread(target=toggle_gpio, args=(self.gpio_pinB, lambda: self.freq_B, "PumpB"))
        self.thread_A.start()
        self.thread_B.start()

        while not self.stop_event.is_set():
            self.pause_event.wait()
            elapsed = time.time() - start_time - self.total_pause_duration
            if elapsed > total_runtime:
                break

            # Interpolate flowrates
            slope = (self.pumpB_max_percent - self.pumpB_min_percent) / total_runtime
            pumpB_percent = self.pumpB_min_percent + slope * elapsed
            pumpB_flow = self.flowrate * (pumpB_percent / 100.0)
            pumpA_flow = self.flowrate - pumpB_flow

            self.freq_A = min(max(pumpA_flow * 18.315, 0), 153)
            self.freq_B = min(max(pumpB_flow * 18.315, 0), 153)

            self.volume_output = (self.flowrate / 60) * elapsed
            print(f"[Gradient] freq_A: {self.freq_A:.2f} Hz, freq_B: {self.freq_B:.2f} Hz")
            print(f"[Gradient] Volume Delivered: {self.volume_output:.2f} ml")

            if round(self.volume_output * 10) / 10.0 > self.last_sent_volume:
                try:
                    self.sock.sendall(f"PumpB_running {self.volume_output:.2f} ml".encode('utf-8'))
                    self.last_sent_volume = round(self.volume_output * 10) / 10.0
                except socket.error as e:
                    print(f"Socket send error: {e}")
                    
            # Check if volume output has reached the target and Stop
            if self.volume_output >= self.total_volume:
                print(f"[Gradient] Target volume {self.total_volume:.2f} ml reached. Stopping pumps.")
                self.stop_event.set()
                break

            time.sleep(0.1)
        print("[Gradient] run() exiting.")
        self.stop()
        

    def pause(self):
        print("Gradient paused")
        self.pause_start_time = time.time()
        self.pause_event.clear()

    def resume(self):
        if self.pause_start_time is not None:
            pause_duration = time.time() - self.pause_start_time
            self.total_pause_duration += pause_duration
            self.pause_start_time = None
        print("Gradient resumed")
        self.pause_event.set()

    def monitor_pump_error(self):
        prev_a = self.gpio_control.monitor_input(24)
        prev_b = self.gpio_control.monitor_input(25)
        while not self.stop_event.is_set():
            curr_a = self.gpio_control.monitor_input(24)
            curr_b = self.gpio_control.monitor_input(25)

            if prev_a == "LOW" and curr_a == "HIGH":
                print("PumpA error detected (GPIO24 HIGH)")
                self.pause()               
                self.data_acquisition.pause()
                self.fraction_collector.pause()
                self.fraction_collector.pause_fraction_collector()
                self.send_pump_error_notification("PumpA")

            elif prev_a == "HIGH" and curr_a == "LOW":
                print("PumpA error cleared (GPIO24 LOW)")
                self.send_pump_error_cleared_notification("PumpA")
                self.resume()
                self.data_acquisition.resume()
                self.fraction_collector.resume()
                self.fraction_collector.resume_fraction_collector()

            if prev_b == "LOW" and curr_b == "HIGH":
                print("PumpB error detected (GPIO25 HIGH)")
                self.pause()
                self.data_acquisition.pause()
                self.fraction_collector.pause()
                self.fraction_collector.pause_fraction_collector()               
                self.send_pump_error_notification("PumpB")

            elif prev_b == "HIGH" and curr_b == "LOW":
                print("PumpB error cleared (GPIO25 LOW)")
                self.send_pump_error_cleared_notification("PumpB")
                self.resume()
                self.data_acquisition.resume()
                self.fraction_collector.resume()
                self.fraction_collector.resume_fraction_collector()

            prev_a, prev_b = curr_a, curr_b
            time.sleep(0.5)
            
    def send_pump_error_notification(self, pump_id):
        try:
            message = f"{pump_id} error"
            self.sock.sendall(message.encode('utf-8'))
            print(f"Error notification sent to server: {message}")
        except socket.error as e:
            print(f"Socket send error ({pump_id} error): {e}")

    def send_pump_error_cleared_notification(self, pump_id):
        try:
            message = f"{pump_id} Error has been cleared"
            self.sock.sendall(message.encode('utf-8'))
            print(f"Error cleared notification sent to server: {message}")
        except socket.error as e:
            print(f"Socket send error ({pump_id} cleared): {e}")

    def stop(self):
        print("[Gradient] stop() called.")
        self.stop_event.set()
        self.gpio_control.set_value(self.gpio_pinA, GPIOControl.LOW)
        self.gpio_control.set_value(self.gpio_pinB, GPIOControl.LOW)
        self.mcp.set_pin_low('A', 1)
        self.mcp.set_pin_high('A', 2)
        self.mcp.set_pin_low('B', 1)
        self.mcp.set_pin_high('B', 2)
        
        print("[Gradient] Joining toggle threads...")
        if self.thread_A and self.thread_A.is_alive():
            self.thread_A.join()
            print("[Gradient] thread_A joined.")
        if self.thread_B and self.thread_B.is_alive():
            self.thread_B.join()
            print("[Gradient] thread_B joined.")
        if self.thread and self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join()
            print("[Gradient] main thread joined.")
                #Stop acquisition and fraction collector if running
        if self.data_acquisition and self.data_acquisition.thread and self.data_acquisition.thread.is_alive():
            self.data_acquisition.stop()
        if self.fraction_collector and self.fraction_collector.thread and self.fraction_collector.thread.is_alive():
            self.fraction_collector.stop()
            self.fraction_collector.stop_fraction_collector()      
            try:          
                self.sock.sendall("PumpB_stopped".encode('utf-8'))
                self.sock.sendall("STOP_SAVE_ACQUISITION".encode('utf-8'))
                print("[Gradient] Stop signals sent to server.")
            except socket.error as e:
                print(f"[Gradient] Socket send error: {e}")    
        self.thread = None
        self.volume_output = 0.0
        self.last_sent_volume = -0.1
        print("[Gradient] All threads joined. Shutdown complete.")


# ---------------- Base Thread ----------------
class BaseThread:
    def __init__(self):
        self.pause_event = threading.Event()
        self.stop_event = threading.Event()
        self.thread = None

    def start(self):
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def run(self):
        while not self.stop_event.is_set():
            self.pause_event.wait()
            self.perform_task()
            time.sleep(0.1)

    def stop(self):
        self.stop_event.set()
        self.pause_event.set()
        if self.thread is not None and self.thread.is_alive():
            if threading.current_thread() != self.thread:
                self.thread.join()
        self.thread = None

    def pause(self):
        self.pause_event.clear()

    def resume(self):
        self.pause_event.set()

    def perform_task(self):
        raise NotImplementedError("Subclasses should implement this method")

# ---------------- Data Acquisition ----------------
class DataAcquisition(BaseThread):
    def __init__(self, adc, gain, sampling_rate, sock, gpio_monitor):
        super().__init__()
        self.adc = adc
        self.gain = gain
        self.sampling_rate = sampling_rate
        self.sock = sock
        self.gpio_monitor = gpio_monitor
        self.flowrate = 0.0
        self.pause_start_time = None
        self.total_pause_duration = 0
        self.frac_collector_running = False
        self.frac_collector_start_time = None

    def set_flowrate(self, flowrate):
        self.flowrate = flowrate

    def perform_task(self):
        start_time = time.time()
        previous_Frac_Mark_value = 0.0
        while not self.stop_event.is_set():
            self.pause_event.wait()
            try:
                value1 = self.adc.read_adc(0, gain=self.gain)
                value2 = self.adc.read_adc(1, gain=self.gain)
                gpio23_status = self.gpio_monitor.monitor_input(23)
            except Exception as e:
                print(f"ADC read error: {e}")
                continue

            elapsed_time = time.time() - start_time - self.total_pause_duration
            eluate_volume = elapsed_time * (self.flowrate / 60)

            if gpio23_status == 'LOW' and not self.frac_collector_running:
                self.frac_collector_running = True
                self.frac_collector_start_time = time.time()
                print("Fraction collector started running")

            if elapsed_time >= 1 and not self.frac_collector_running:
                print("Fraction collector not running")

            if gpio23_status == 'LOW' and previous_Frac_Mark_value == 0.0:
                Frac_Mark = 1.0
                previous_Frac_Mark_value = Frac_Mark
            else:
                Frac_Mark = 0.0
                previous_Frac_Mark_value = Frac_Mark

            data = f"{value1},{value2},{elapsed_time},{eluate_volume},{Frac_Mark}"
            try:
                self.sock.sendall(data.encode('utf-8'))
            except socket.error as e:
                print(f"Socket send error: {e}")
                break

            print(f"Elapsed_Time: {elapsed_time:.2f} sec, Eluate_Volume: {eluate_volume:.4f} mls, Chan1: {value1}, Chan2: {value2}, GPIO23: {gpio23_status}")
            time.sleep(1 / self.sampling_rate)

    def pause(self):
        self.pause_start_time = time.time()
        self.pause_event.clear()

    def resume(self):
        if self.pause_start_time is not None:
            pause_duration = time.time() - self.pause_start_time
            self.total_pause_duration += pause_duration
            self.pause_start_time = None
        self.pause_event.set()

# ---------------- Fraction Collector ----------------
class FractionCollector(BaseThread):
    def __init__(self, gpio_control, sock):
        super().__init__()
        self.gpio_control = gpio_control
        self.sock = sock
        self.error_detected = False
        self.pins = {
            'operable_in': 5,
            'start_stop': 6,
            'pause_resume': 22,
            'operable_out': 16,
            'error': 26
        }

    def perform_task(self):
        self.monitor_error()
        if self.error_detected:
            self.monitor_error_cleared()

    def set_pin(self, pin_name, value):
        self.gpio_control.set_value(self.pins[pin_name], value)

    def get_pin_status(self, pin_name):
        return self.gpio_control.monitor_input(self.pins[pin_name])

    def print_status(self, pin_name):
        status = self.get_pin_status(pin_name)
        print(f"GPIO{self.pins[pin_name]} status: {status}")

    def monitor_error(self):
        if self.get_pin_status('error') == "LOW":
            print("Fraction Collector ERROR has occurred")
            self.print_status('error')
            self.set_pin('start_stop', GPIOControl.HIGH)
            pump_a_flowrate_controller.pause()
            data_acquisition.pause()
            self.send_error_notification()
            self.error_detected = True

    def monitor_error_cleared(self):
        while self.get_pin_status('error') == "LOW":
            time.sleep(1)
        print("Fraction Collector Error has been cleared")
        self.send_error_cleared_notification()
        pump_a_flowrate_controller.resume()
        data_acquisition.resume()
        self.set_pin('start_stop', GPIOControl.LOW)
        self.error_detected = False 
        if self.thread is None or not self.thread.is_alive():
            self.start()
        

    def send_error_notification(self):
        try:
            self.sock.sendall("Fraction Collector error".encode('utf-8'))
            print("Error notification sent to server")
        except socket.error as e:
            print(f"Socket send error: {e}")

    def send_error_cleared_notification(self):
        try:
            self.sock.sendall("Fraction Collector Error has been cleared".encode('utf-8'))
            print("Error cleared notification sent to server")
        except socket.error as e:
            print(f"Socket send error: {e}")

    def start_fraction_collector(self):
        self.set_pin('start_stop', GPIOControl.LOW)
        self.print_status('operable_out')

    def stop_fraction_collector(self):
        if self.get_pin_status('operable_out') == "LOW":
            self.set_pin('start_stop', GPIOControl.HIGH)
            print("Fraction collector stopped")

    def pause_fraction_collector(self):
        if self.get_pin_status('operable_out') == "LOW" and self.get_pin_status('start_stop') == "LOW":
            self.set_pin('pause_resume', GPIOControl.LOW)
            print("Fraction collector paused")

    def resume_fraction_collector(self):
        if (self.get_pin_status('operable_out') == "LOW" and
            self.get_pin_status('start_stop') == "LOW" and
            self.get_pin_status('pause_resume') == "LOW"):
            self.set_pin('pause_resume', GPIOControl.HIGH)
            print("Fraction collector resumed")

# ---------------- Error Detection ----------------

class ErrorDetection(BaseThread):
    def __init__(self, gpio_control, error_pins, sock):
        super().__init__()
        self.gpio_control = gpio_control
        self.error_pins = error_pins
        self.sock = sock
        self.error_active = {}      
        self.previous_state = {}


        for pin_name, pin_num in self.error_pins.items():
            state = self.gpio_control.monitor_input(pin_num)
            self.previous_state[pin_name] = state
            if state == "HIGH":
                print(f"[Startup] {pin_name} already in error state (GPIO{pin_num} HIGH)")
                self.handle_error(pin_name)
                
    def perform_task(self):
        for pin_name, pin_num in self.error_pins.items():
            current_state = self.gpio_control.monitor_input(pin_num)
            prev_state = self.previous_state[pin_name]
            print(f"[Debug] {pin_name} GPIO{pin_num} state: {current_state} (prev: {prev_state})")
            # Detect LOW to HIGH transition
            if prev_state == "LOW" and current_state == "HIGH":
                print(f"[ErrorDetection] {pin_name} error has occurred (GPIO{pin_num} HIGH)")
                self.handle_error(pin_name)
            # Detect HIGH to LOW transition
            elif prev_state == "HIGH" and current_state == "LOW":
                print(f"[ErrorDetection] {pin_name} error has cleared (GPIO{pin_num} LOW)")
                self.handle_error_cleared(pin_name)
            # Update previous state
            self.previous_state[pin_name] = current_state

    def handle_error(self, pin_name):
        if pin_name == 'pump_a_error':
            pump_a_flowrate_controller.pause()
            fraction_collector.pause()
            data_acquisition.pause()
            self.send_error_notification(f"{pin_name} occurred")

    def handle_error_cleared(self, pin_name):
        if pin_name == 'pump_a_error':
            pump_a_flowrate_controller.resume()
            fraction_collector.resume()
            data_acquisition.resume()
            self.send_error_cleared_notification(f"{pin_name} cleared")

    def send_error_notification(self, message):
        try:
            self.sock.sendall(f"ERROR: {message}".encode('utf-8'))
            print(f"Sent error notification: {message}")
        except socket.error as e:
            print(f"Socket send error: {e}")

    def send_error_cleared_notification(self, message):
        try:
            self.sock.sendall(f"ERROR_CLEARED: {message}".encode('utf-8'))
            print(f"Sent error cleared notification: {message}")
        except socket.error as e:
            print(f"Socket send error: {e}")

         


# ---------------- Heartbeat handling ----------------
class HeartbeatSender(threading.Thread):
    def __init__(self, sock, interval=5):
        super().__init__(daemon=True)
        self.sock = sock
        self.interval = interval
        self.running = True

    def run(self):
        while self.running:
            try:
                self.sock.sendall("HEARTBEAT".encode('utf-8'))
                print("[Client] Sent HEARTBEAT")
            except socket.error as e:
                print(f"[Client] Heartbeat send error: {e}")
                break
            time.sleep(self.interval)

    def stop(self):
        self.running = False

# ---------------- Thread Health Monitor ----------------
def monitor_thread_health(thread, name):
    def monitor():
        while True:
            if thread.thread and thread.thread.is_alive():
                print(f"[Monitor] {name} thread is alive.")
            else:
                print(f"[Monitor] {name} thread is NOT running! Restarting...")
                thread.start()
                time.sleep(10)# Check every 10 seconds
                threading.Thread(target=monitor, daemon=True).start()

# ---------------- Main Script ----------------
if __name__ == '__main__':
    adc = Adafruit_ADS1x15.ADS1115(busnum=1)
    GAIN = 16
    sampling_rate = 10
    server_ip = '128.104.117.228'
    server_port = 5000

    # Connect to server
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((server_ip, server_port))
            print("Connected to server")
            heartbeat_sender = HeartbeatSender(sock)
            heartbeat_sender.start()
                      

            gpio_configs = {
                5: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},
                6: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},
                22: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},
                23: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},
                24: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP}, # PumpA error
                16: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},
                26: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},
                17: {'direction': Direction.OUTPUT, 'initial_state': 'LOW'},
                13: {'direction': Direction.OUTPUT, 'initial_state': 'LOW'},# PumpA flowrate control
                4: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},
                17: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},
                27: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},
                25: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP}, # PumpB error
                12: {'direction': Direction.OUTPUT, 'initial_state': 'LOW'} # PumpB flowrate control
            }

            gpio_monitor = GPIOControl('gpiochip0', gpio_configs)
            #gpio_monitor.set_value(17, GPIOControl.LOW)
            gpio_monitor.set_value(5, GPIOControl.LOW)
            # GPIO pin assignments for flowrate control
            gpio_pinA = 13 # GPIO13 for PumpA flowrate control
            gpio_pinB = 12 # GPIO12 for PumpB flowrate control
            

            data_acquisition = DataAcquisition(adc, GAIN, sampling_rate, sock, gpio_monitor)
            fraction_collector = FractionCollector(gpio_monitor, sock)
 
            bus = SMBus(1)
            mcp = MCP23017(bus, 0x21)
            mcp.set_pin_high('A', 0)# Set GPA0 HIGH to keep PumpA wash program inactive at startup
            mcp.set_pin_high('A', 2)# Set GPA2 HIGH sets PumpA speed control to internal
            mcp.set_pin_high('B', 0)# Set GPB0 HIGH to keep PumpB wash program inactive at startup
            mcp.set_pin_high('B', 2)# Set GPB2 HIGH sets PumpB speed control to internal

            valve_controller = MV7ValveController(gpio_monitor, mcp)          
            pump_a_flowrate_controller = PumpAFlowrateController(
                gpio_monitor, gpio_pinA, sock, mcp, data_acquisition, fraction_collector
            )
            
            gradient_flowrate_controller = GradientFlowrateController(
                gpio_monitor, gpio_pinA, gpio_pinB, sock, mcp, data_acquisition, fraction_collector
            )
           
            #Define GPIO-based error pins
            error_pins = {'pump_a_error': 24}
                        
            error_detection = ErrorDetection(
                gpio_monitor, error_pins, sock
            )
            error_detection.start()
            monitor_thread_health(error_detection, "ErrorDetection")

            break
        except socket.error:
            print("Client trying to connect to server...")
            time.sleep(2)

    try:
        while True:
            signal = sock.recv(1024).decode('utf-8')

            if signal.startswith("FLOWRATE:"):
                flowrate_value = float(signal.split(":")[1])
                data_acquisition.set_flowrate(flowrate_value)
                pump_a_flowrate_controller.set_flowrate(flowrate_value)
                print(f"Received flowrate: {flowrate_value} ml/min")

            elif signal.startswith("PumpA_Volume:"):
                PumpA_volume_value = float(signal.split(":")[1])
                pump_a_flowrate_controller.set_volume_value(PumpA_volume_value)
                print(f"Received PumpA volume: {PumpA_volume_value} ml")

            elif signal.startswith("WASH_PUMPS_JSON:"):
                try:
                    wash_command = json.loads(signal.split(":", 1)[1])
                    pumps = wash_command.get("WASH_PUMPS", [])
                    solvent_exchange = SolventExchangePumps(mcp, sock, valve_controller)
                    threading.Thread(target=solvent_exchange.start_wash, args=(pumps,),
                    daemon=True).start()
                except json.JSONDecodeError as e:
                    print(f"Error decoding WASH_PUMPS_JSON: {e}")

            elif signal == "START_PUMPS":
                print("Received start_pumps signal")
                mcp.set_pin_high('A', 1)# GPA1 HIGH = Run
                mcp.set_pin_low('A', 2)# GPA2 LOW = Enable external speed control
                pump_a_flowrate_controller.start()
                monitor_thread_health(pump_a_flowrate_controller, "PumpAFlowrateController")

            elif signal == "STOP_PUMPS":
                print("Received stop_pumps signal")
                mcp.set_pin_low('A', 1)# GPA1 LOW = Standby
                mcp.set_pin_high('A', 2)# GPA2 HIGH = Disable external speed control
                pump_a_flowrate_controller.stop()

            elif signal == "START_ADC":
                print("Received START_ADC signal")
                data_acquisition.stop_event.clear()
                data_acquisition.resume()
                data_acquisition.start()
                fraction_collector.stop_event.clear()
                fraction_collector.resume()
                fraction_collector.start()
                fraction_collector.start_fraction_collector()
                error_detection.start()
                monitor_thread_health(data_acquisition, "DataAcquisition")
                monitor_thread_health(fraction_collector, "FractionCollector")

            elif signal == "STOP_ADC":
                print("Received STOP_ADC signal")
                data_acquisition.stop()
                data_acquisition.frac_collector_running = False
                data_acquisition.frac_collector_start_time = None
                fraction_collector.stop()
                fraction_collector.stop_fraction_collector()
                error_detection.stop()

            elif signal == "PAUSE_ADC":
                print("Received PAUSE_ADC signal")
                data_acquisition.pause()
                fraction_collector.pause()
                fraction_collector.pause_fraction_collector()
                pump_a_flowrate_controller.pause()
                gradient_flowrate_controller.pause()

            elif signal == "RESUME_ADC":
                print("Received RESUME_ADC signal")
                data_acquisition.resume()
                fraction_collector.resume()
                fraction_collector.resume_fraction_collector()
                pump_a_flowrate_controller.resume()
                gradient_flowrate_controller.resume()
                
            elif signal.startswith("System_Valve_Position:"):
                valve_position = signal.split(":")[1]
                print(f"Received System Valve Position: {valve_position}")
                try:
                    valve_controller.move_to_position(valve_position)
                except Exception as e:
                    print(f"Valve movement error: {e}")
                    

            elif signal.startswith("ISOCRATIC_RUN_METHOD_JSON:"):
                try:
                    run_method = json.loads(signal.split(":", 1)[1])

                    # Extract and apply parameters
                    flowrate = float(run_method.get("FLOWRATE", 0))
                    volume = float(run_method.get("PumpA_Volume", 0))
                    valve_position = run_method.get("System_Valve_Position", "LOAD")
                    start_pumps = run_method.get("START_PUMPS", False)
                    start_adc = run_method.get("START_ADC", False)

                    print(f"Received ISOCRATIC_RUN_METHOD_JSON: FLOWRATE={flowrate}, Volume={volume}, Valve={valve_position}, START_PUMPS={start_pumps}, START_ADC={start_adc}")

                    # Apply flowrate and volume
                    data_acquisition.set_flowrate(flowrate)
                    pump_a_flowrate_controller.set_flowrate(flowrate)
                    pump_a_flowrate_controller.set_volume_value(volume)

                    # Move valve
                    try:
                        valve_controller.move_to_position(valve_position)
                    except Exception as e:
                        print(f"Valve movement error: {e}")

                    # Start pumps
                    if start_pumps:
                        mcp.set_pin_high('A', 1)# GPA1 HIGH = Run
                        mcp.set_pin_low('A', 2)# GPA2 LOW = Enable external speed control
                        pump_a_flowrate_controller.start()
                        monitor_thread_health(pump_a_flowrate_controller, "PumpAFlowrateController")

                    # Start ADC and fraction collector
                    if start_adc:
                        data_acquisition.stop_event.clear()
                        data_acquisition.resume()
                        data_acquisition.start()
                        fraction_collector.stop_event.clear()
                        fraction_collector.resume()
                        fraction_collector.start()
                        fraction_collector.start_fraction_collector()
                        error_detection.start()
                        monitor_thread_health(data_acquisition, "DataAcquisition")
                        monitor_thread_health(fraction_collector, "FractionCollector")

                except json.JSONDecodeError as e:
                    print(f"Error decoding ISOCRATIC_RUN_METHOD_JSON: {e}")
                    
            
            elif signal.startswith("GRADIENT_RUN_METHOD_JSON:"):
                try:
                    run_method = json.loads(signal.split(":", 1)[1])
                    flowrate = float(run_method.get("FLOWRATE", 0))
                    gradient_volume = float(run_method.get("GRADIENT_VOLUME", 0))
                    pumpB_min_percent = float(run_method.get("PumpB_min_percent", 0))
                    pumpB_max_percent = float(run_method.get("PumpB_max_percent", 100))
                    valve_position = run_method.get("System_Valve_Position", "LOAD")
                    start_pumps = run_method.get("START_PUMPS", False)
                    start_adc = run_method.get("START_ADC", False)
                    
                    print(f"[Gradient JSON] FLOWRATE={flowrate}, VOLUME={gradient_volume}, "
                        f"PumpB_min={pumpB_min_percent}%, PumpB_max={pumpB_max_percent}%, "
                        f"Valve={valve_position}, START_PUMPS={start_pumps}, START_ADC={start_adc}")

                    # Store parameters for gradient execution
                    data_acquisition.set_flowrate(flowrate)
                    gradient_flowrate_controller.set_flowrate(flowrate)
                    gradient_flowrate_controller.set_volume_value(gradient_volume)
                    gradient_flowrate_controller.set_gradient_profile(pumpB_min_percent, pumpB_max_percent)

                    try:
                        valve_controller.move_to_position(valve_position)
                    except Exception as e:
                        print(f"Valve movement error: {e}")

                    if start_pumps:
                        # Enable PumpA (external speed control)
                        mcp.set_pin_high('A', 1) # GPA1 HIGH = Run pumpA
                        mcp.set_pin_low('A', 2) # GPA2 LOW = Enable external speed control pumpA
                        # Enable PumpB (external speed control)
                        mcp.set_pin_high('B', 1) # GPB1 HIGH = Run pumpB
                        mcp.set_pin_low('B', 2) # GPB2 LOW = Enable external speed control pumpB
                        gradient_flowrate_controller.start()
                        monitor_thread_health(gradient_flowrate_controller, "GradientFlowrateController")

                    if start_adc:
                        data_acquisition.stop_event.clear()
                        data_acquisition.resume()
                        data_acquisition.start()
                        fraction_collector.stop_event.clear()
                        fraction_collector.resume()
                        fraction_collector.start()
                        fraction_collector.start_fraction_collector()
                        error_detection.start()
                        monitor_thread_health(data_acquisition, "DataAcquisition")
                        monitor_thread_health(fraction_collector, "FractionCollector")
                except json.JSONDecodeError as e:
                    print(f"Error decoding GRADIENT_RUN_METHOD_JSON: {e}")


            elif signal == "HEARTBEAT":
                error_detection.handle_heartbeat()
                
            elif signal.startswith("METHOD_STOP_JSON:"):
                try:
                    stop_method = json.loads(signal.split(":", 1)[1])
                    if stop_method.get("STOP_PUMPS"):
                        print("Stopping pumps as per METHOD_STOP_JSON")
                        mcp.set_pin_low('A', 1)# GPA1 LOW = Standby
                        mcp.set_pin_high('A', 2)# GPA2 HIGH = Disable external speed control
                        mcp.set_pin_low('B', 1)# GPB1 LOW = Standby
                        mcp.set_pin_high('B', 2)# GPB2 HIGH = Disable external speed control
                        pump_a_flowrate_controller.stop()
                        gradient_flowrate_controller.stop()

                    valve_position = stop_method.get("System_Valve_Position", "LOAD")
                    flowrate = float(stop_method.get("FLOWRATE", 0.0))
                    volume = float(stop_method.get("PumpA_Volume", 0.0))
                    volume = float(stop_method.get("PumpB_Volume", 0.0))
                    stop_adc = stop_method.get("STOP_ADC", False)

                    print(f"Resetting valve to {valve_position}, flowrate to {flowrate}, volume to {volume}")
                    data_acquisition.set_flowrate(flowrate)
                    pump_a_flowrate_controller.set_flowrate(flowrate)
                    pump_a_flowrate_controller.set_volume_value(volume)
                    gradient_flowrate_controller.set_flowrate(flowrate)
                    gradient_flowrate_controller.set_volume_value(volume)
                    if stop_adc:
                        data_acquisition.stop()
                        data_acquisition.frac_collector_running = False
                        data_acquisition.frac_collector_start_time = None
                        fraction_collector.stop()
                        fraction_collector.stop_fraction_collector()
                        error_detection.stop()
                    
                    try:
                        valve_controller.move_to_position(valve_position)
                    except Exception as e:
                        print(f"Valve movement error: {e}")

                # Optional: Add GPIO/MCP logic to move valve to LOAD if needed
                except json.JSONDecodeError as e:
                    print(f"Error decoding METHOD_STOP_JSON: {e}")
            

    #except socket.error as e:
        #print(f"Socket error: {e}")
                    
    
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down gracefully...")

    finally:
        heartbeat_sender.stop()
        data_acquisition.stop()
        fraction_collector.stop()
        fraction_collector.stop_fraction_collector()
        valve_controller.cleanup()
        error_detection.stop()
        gpio_monitor.clear_gpio()
        gpio_monitor.set_value(17, GPIOControl.HIGH) # Ensure valve motor stays OFF
        gpio_monitor.set_value(13, GPIOControl.LOW)
        gpio_monitor.set_value(12, GPIOControl.LOW)
        mcp.clear_all_outputs()
        sock.close()
        