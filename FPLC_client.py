
#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
FPLC_client_0.2.5 (FPLC_controller) - Updated to control PumpA flowrate and run

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

        # Set GPA0, GPA1, GPA2 as outputs (bits 0?2 = 0), GPA7 as input (bit 7 = 1)
        self.bus.write_byte_data(self.address, self.IODIRA, 0b10000000)

        # Enable pull-up on GPA7 (bit 7 = 1)
        self.bus.write_byte_data(self.address, self.GPPUA, 0b10000000)

        
        self.set_pin_high('A', 0) #initialize GPA0 to HIGH (inactive wash)
        self.set_pin_high('A', 2) #initialize GPA2 to HIGH (internal speed control)

        # Set all A and B pins as outputs
        #self.bus.write_byte_data(self.address, self.IODIRA, 0x00)
        #self.bus.write_byte_data(self.address, self.IODIRB, 0x00)

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



# ---------------- PumpA Control ----------------

class PumpA:
    def __init__(self, mcp, sock):
        self.mcp = mcp
        self.sock = sock
        self.port = 'A'
        self.pin_wash = 0# GPA0 controls wash start
        self.pin_status = 7# GPA7 monitors wash status

    def start_wash(self):
        print("PumpA wash started")
        self.mcp.set_pin_low(self.port, self.pin_wash)# Activate wash
        if self.mcp.read_pin(self.port, self.pin_status) == 0:
            print("pumpA wash program activated")

    def monitor_wash(self):
        # Continuously monitor GPA7 while GPA0 is LOW
        while self.mcp.read_pin(self.port, self.pin_wash) == 0:
            if self.mcp.read_pin(self.port, self.pin_status) == 1:
                self.mcp.set_pin_high(self.port, self.pin_wash)# Deactivate wash
                print("pumpA wash program complete")
                try:
                    self.sock.sendall("PUMP_A_WASH_COMPLETED".encode('utf-8'))
                    print("Sent wash completion message to server")
                except socket.error as e:
                    print(f"Socket send error: {e}")
                break
            time.sleep(0.5)# Polling interval


# ---------------- PumpA Flowrate Controller ----------------
class PumpAFlowrateController:
    def __init__(self, gpio_control, gpio_pin, mcp):
        self.gpio_control = gpio_control
        self.gpio_pin = gpio_pin
        self.mcp = mcp
        self.flowrate = 0.0
        self.thread = None
        self.stop_event = threading.Event()
        self.pump_runtime = 0.0
        self.volume_output = 0.0
        self.PumpA_volume_value = 0.0

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

    def run(self):
        start_time = time.time()
        while not self.stop_event.is_set():
            period = self.calculate_period()
            if period is None:
                self.gpio_control.set_value(self.gpio_pin, GPIOControl.LOW)
                time.sleep(0.1)
                continue
            half_period = period / 2.0
            self.gpio_control.set_value(self.gpio_pin, GPIOControl.HIGH)
            time.sleep(half_period)
            self.gpio_control.set_value(self.gpio_pin, GPIOControl.LOW)
            time.sleep(half_period)
            print(f"Toggling GPIO{self.gpio_pin} at {1/period:.2f} Hz")
            
            # Calculate runtime and volume output
            self.pump_runtime = time.time() - start_time
            self.volume_output = (self.flowrate/60) * self.pump_runtime

            # Check if volume output has reached the target
            if self.volume_output >= self.PumpA_volume_value:
                print(f"Volume output {self.volume_output:.2f} ml has reached the target {self.PumpA_volume_value:.2f} ml")
                self.mcp.set_pin_low('A', 1)# GPA1 LOW
                self.mcp.set_pin_high('A', 2)# GPA2 HIGH
                self.gpio_control.set_value(self.gpio_pin, GPIOControl.LOW)# GPIO13 LOW
                self.stop()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            if threading.current_thread() != self.thread:
                self.thread.join()
        self.gpio_control.set_value(self.gpio_pin, GPIOControl.LOW)
        self.thread = None


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
            self.stop_fraction_collector()
            self.stop()
            self.set_pin('start_stop', GPIOControl.HIGH)
            data_acquisition.stop()
            self.send_error_notification()
            self.error_detected = True

    def monitor_error_cleared(self):
        while self.get_pin_status('error') == "LOW":
            time.sleep(1)
        print("Fraction Collector Error has been cleared")
        self.send_error_cleared_notification()
        self.error_detected = False

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
        
        for pin_name, pin_num in self.error_pins.items():
            state = self.gpio_control.monitor_input(pin_num)
            self.error_active[pin_name] = (state == "LOW")
            self.previous_error_state = state

            if state == "LOW":
                print(f"[Startup] {pin_name} already in error state (GPIO{pin_num} LOW)")
                self.handle_error(pin_name)

    def perform_task(self):
        print("[ErrorDetection] perform_task running...")
        for pin_name, pin_num in self.error_pins.items():
            current_state = self.gpio_control.monitor_input(pin_num)
            if current_state == "LOW" and not self.error_active[pin_name]:
                print(f"[ErrorDetection] {pin_name} error has occurred")
                self.handle_error(pin_name)
                self.error_active[pin_name] = True
            elif current_state == "HIGH" and self.error_active[pin_name]:
                print(f"[ErrorDetection] {pin_name} error has cleared")
                self.send_error_cleared_notification()
                self.error_active[pin_name] = False

    def handle_error(self, pin_name):
        if pin_name == 'fraction_collector_error':
            fraction_collector.stop_fraction_collector()
            fraction_collector.stop()
            fraction_collector.set_pin('start_stop', GPIOControl.HIGH)
            data_acquisition.stop()
            self.send_error_notification()
            self.resume()

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
                16: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},
                26: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},
                17: {'direction': Direction.OUTPUT, 'initial_state': 'LOW'},
                13: {'direction': Direction.OUTPUT, 'initial_state': 'LOW'},
                4: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},
                17: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},
                27: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'}
            }

            gpio_monitor = GPIOControl('gpiochip0', gpio_configs)
            #gpio_monitor.set_value(17, GPIOControl.LOW)
            gpio_monitor.set_value(5, GPIOControl.LOW)
            

            data_acquisition = DataAcquisition(adc, GAIN, sampling_rate, sock, gpio_monitor)
            fraction_collector = FractionCollector(gpio_monitor, sock)
            error_pins = {'fraction_collector_error': 26}
            error_detection = ErrorDetection(gpio_monitor, error_pins, sock)
            error_detection.start()
            monitor_thread_health(error_detection, "ErrorDetection")

            bus = SMBus(1)
            mcp = MCP23017(bus, 0x21)
            mcp.set_pin_high('A', 0)# Set GPA0 HIGH to keep wash program inactive at startup
            mcp.set_pin_high('A', 2)# Set GPA2 HIGH to keep wash program inactive at startup

            pump_a = PumpA(mcp, sock)
            pump_a_flowrate_controller = PumpAFlowrateController(gpio_monitor, 13, mcp)

            valve_controller = MV7ValveController(gpio_monitor, mcp)

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

            elif signal == "WASH_PUMP_A":
                print("Received Wash_PumpA signal")
                pump_a.start_wash()
                pump_a.monitor_wash()
                #time.sleep(10)  # Duration of wash cycle
                #pump_a.stop_wash()

            elif signal == "WASH_PUMP_B":
                print("Received Wash_PumpB signal")

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

            elif signal == "RESUME_ADC":
                print("Received RESUME_ADC signal")
                data_acquisition.resume()
                fraction_collector.resume()
                fraction_collector.resume_fraction_collector()
                
            elif signal.startswith("System_Valve_Position:"):
                valve_position = signal.split(":")[1]
                print(f"Received System Valve Position: {valve_position}")
                try:
                    valve_controller.move_to_position(valve_position)
                except Exception as e:
                    print(f"Valve movement error: {e}")

            elif signal == "HEARTBEAT":
                error_detection.handle_heartbeat()

            #fraction_collector.monitor_error()

    except socket.error as e:
        print(f"Socket error: {e}")

    finally:
        heartbeat_sender.stop()
        data_acquisition.stop()
        fraction_collector.stop()
        fraction_collector.stop_fraction_collector()
        valve_controller.cleanup()
        error_detection.stop()
        gpio_monitor.clear_gpio()
        gpio_monitor.set_value(13, GPIOControl.LOW)
        mcp.clear_all_outputs()
        sock.close()
        