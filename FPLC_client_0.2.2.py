#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
>>>>>>>>>>FPLC_client_0.2.2 (FPLC_controller)
working with FPLC_server_GUI_0.3.1.8.7
Functions:
1) ADS1115 ADC data acquisition
2) communicate with FPLC_server_GUI_0.3.14
3) Threading 
4) GPIO communication 042725
5) Added Fraction Collector signal handling 050125, add threading to Fraction Collector
6) add error handling thread
7) added pause/resume handling to data acquisition
8) add mcp23017 chip handling
'''

import socket
import threading
import time
import Adafruit_ADS1x15
import gpiod
from gpiod.line import Direction, Value, Bias
from smbus2 import SMBus



class GPIOControl:
    '''
    Note to self: gpiod defines .ACTIVE as "1" and .INACTIVE as "0".
    However the active state for input/output pins on pharmacia pumps
    and fraction collector is "0".  Thus to avoid confusion I will use "HIGH" and "LOW"
    to define controller states
    '''
    
    HIGH = Value.ACTIVE  
    LOW = Value.INACTIVE

    def __init__(self, chip_name, gpio_configs):
        super().__init__()
        self.chip_name = chip_name
        self.gpio_configs = gpio_configs
        self.request = {}
        self.setup_gpio()

    def setup_gpio(self):
        for line_num, config in self.gpio_configs.items():
            initial_value = self.HIGH if config.get('initial_state') == 'HIGH' else self.LOW
            line_settings = gpiod.LineSettings(
                direction=config['direction'],
                output_value=self.HIGH if config['direction'] == Direction.OUTPUT else self.LOW,
                bias=config.get('bias', Bias.DISABLED)
            )
            self.request[line_num] = gpiod.request_lines(
                f"/dev/{self.chip_name}",
                consumer="gpio_control",
                config={line_num: line_settings}
            )

    def set_value(self, line_num, value):
        self.request[line_num].set_value(line_num, value)

    def get_value(self, line_num):
        return self.request[line_num].get_value(line_num)

    def monitor_input(self, line_num):
        value = self.get_value(line_num)
        return "HIGH" if value == self.HIGH else "LOW"


class MCP23017:
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
        self.IODIRA = 0x00# I/O direction register for port A
        self.IODIRB = 0x01# I/O direction register for port B
        self.OLATA = 0x14# Output latch register for port A
        self.OLATB = 0x15# Output latch register for port B

        # Set all A and B pins as outputs
        self.bus.write_byte_data(self.address, self.IODIRA, 0x00)
        self.bus.write_byte_data(self.address, self.IODIRB, 0x00)

    def set_pin_high(self, port, pin):
        if port == 'A':
            current_value = self.bus.read_byte_data(self.address, self.OLATA)
            new_value = current_value | (1 << pin)
            self.bus.write_byte_data(self.address, self.OLATA, new_value)
        elif port == 'B':
            current_value = self.bus.read_byte_data(self.address, self.OLATB)
            new_value = current_value | (1 << pin)
            self.bus.write_byte_data(self.address, self.OLATB, new_value)

# Initialize the I2C bus and MCP23017
bus = SMBus(1)
mcp = MCP23017(bus, 0x21)

# Set GPA0 and GPB0 to HIGH
mcp.set_pin_high('A', 0)
mcp.set_pin_high('B', 0)


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
            time.sleep(0.1)# Adjust the sleep time as needed

    def stop(self):
        self.stop_event.set()
        self.pause_event.set()# Ensure the thread can exit
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

    def set_flowrate(self, flowrate):
        self.flowrate = flowrate

    def perform_task(self):
        start_time = time.time()
        previous_Frac_Mark_value = 0.0
        while not self.stop_event.is_set():
            self.pause_event.wait()
            print (f'time paused:{self.total_pause_duration}')
            try:
                value1 = self.adc.read_adc(0, gain=self.gain)
                value2 = self.adc.read_adc(1, gain=self.gain)
                gpio12_status = self.gpio_monitor.monitor_input(12)
            except Exception as e:
                print(f"ADC read error: {e}")
                continue
            elapsed_time = time.time() - start_time - self.total_pause_duration
            eluate_volume = elapsed_time * (self.flowrate/60)
            if (gpio12_status == 'LOW' and previous_Frac_Mark_value == 0.0):
                Frac_Mark = 1.0
                previous_Frac_Mark_value = Frac_Mark
            else:
                Frac_Mark = 0.0
                previous_Frac_Mark_value = Frac_Mark
                
            data = f"{value1},{value2},{elapsed_time},{eluate_volume},{Frac_Mark}"
            #data = f"{value1},{value2},{run_volume},{Frac_Mark}"
            try:
                self.sock.sendall(data.encode('utf-8'))
            except socket.error as e:
                print(f"Socket send error: {e}")
                break
            print(f"Elapsed_Time: {elapsed_time: .2f} sec, Eluate_Volume: {eluate_volume:.4f} mls, Chan1: {value1}, Chan2: {value2}, GPIO12: {gpio12_status}")
            time.sleep(1 / self.sampling_rate)

    def pause(self):
        self.pause_start_time = time.time()# Record the start time of the pause
        self.pause_event.clear()# Clear the pause event to pause execution

    def resume(self):
        if self.pause_start_time is not None:
            pause_duration = time.time() - self.pause_start_time
            self.total_pause_duration += pause_duration
            self.pause_start_time = None# Reset for future pauses
        self.pause_event.set()# Resume execution


class FractionCollector(BaseThread):
    def __init__(self, gpio_control, sock):
        super().__init__()
        self.gpio_control = gpio_control
        self.sock = sock
        self.error_detected = False
        self.pins = {
        'operable_in': 5,
        'start_stop': 6,
        'pause_resume': 13,
        'operable_out': 16,
        'error': 26
        }

    def perform_task(self):
        self.monitor_error()
        if self.error_detected:
            self.monitor_error_cleared()        
        # add other operations that the fraction collector needs to perform regularly

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
            self.send_error_notification() # Send notification to server
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
            #self.print_status('operable_out')

    def pause_fraction_collector(self):
        if self.get_pin_status('operable_out') == "LOW" and self.get_pin_status('start_stop') == "LOW":
            self.set_pin('pause_resume', GPIOControl.LOW)
            print("Fraction collector paused")

    def resume_fraction_collector(self):
        if (self.get_pin_status('operable_out') == "LOW" and self.get_pin_status('start_stop') == "LOW" and self.get_pin_status('pause_resume') == "LOW"):
            self.set_pin('pause_resume', GPIOControl.HIGH)
            print("Fraction collector resumed")
           

class ErrorDetection(BaseThread):
    def __init__(self, gpio_control, error_pins):
        super().__init__()
        self.gpio_control = gpio_control
        self.error_pins = error_pins

    def perform_task(self):
        for pin_name, pin_num in self.error_pins.items():
            if self.gpio_control.monitor_input(pin_num) == "LOW":
                print(f"{pin_name} error has occurred")
                self.handle_error(pin_name)


    def handle_error(self, pin_name):
        if pin_name == 'fraction_collector_error':
            # Stop the fraction collector immediately
            fraction_collector.stop_fraction_collector()
            fraction_collector.stop()
        # Add error handling for other hardware here
            fraction_collector.set_pin('start_stop', GPIOControl.HIGH)
            data_acquisition.stop()
            fraction_collector.send_error_notification() # Send notification to server


# Main Script
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
            break
        except socket.error as e:
            print(f"Client trying to connect to server...")
            #print(f"Socket error: {e}. Retrying...")
            time.sleep(2)

    gpio_configs = {
        5: {'direction': Direction.OUTPUT, 'initial_state': 'LOW'}, # Enable FPLC device "Operable(in)"
        6: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'}, # Frac Collector "START/STOP"
        13: {'direction': Direction.OUTPUT, 'initial_state': 'HIGH'},  # Pause/Resume
        12: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP}, # Frac Collector "Event Mark"
        16: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP},  # Operable (out)
        26: {'direction': Direction.INPUT, 'bias': Bias.PULL_UP}, # Frac Collector Error
        17: {'direction': Direction.OUTPUT},# TEST LED TOGGLE
        # Add more GPIO configurations here
    }    
    gpio_monitor = GPIOControl('gpiochip0', gpio_configs)
    gpio_monitor.set_value(5, GPIOControl.LOW) #set frac collector operable pin10 to "0"
    data_acquisition = DataAcquisition(adc, GAIN, sampling_rate, sock, gpio_monitor)
    
    # Instantiate device-specific classes
    fraction_collector = FractionCollector(gpio_monitor, sock)
    #pump_a = PumpA(gpio_monitor)
    
    error_pins = {
        'fraction_collector_error': 26,
        # Add other error pins here
    }
    error_detection = ErrorDetection(gpio_monitor, error_pins)

    try:
        while True:
            signal = sock.recv(1024).decode('utf-8')
            if signal.startswith("FLOWRATE:"):
                flowrate_value = float(signal.split(":")[1])
                data_acquisition.set_flowrate(flowrate_value)
                print(f"Received flowrate: {flowrate_value} ml/min")
            
            elif signal == "START_ADC":
                print("Received START_ADC signal")
                data_acquisition.stop_event.clear()  # Ensure the stop event is cleared
                data_acquisition.resume()  # Ensure the pause event is set to resume
                data_acquisition.start()
                fraction_collector.stop_event.clear()
                fraction_collector.resume()
                fraction_collector.start()
                fraction_collector.start_fraction_collector()
                error_detection.start()
                #pump_a.start()
            elif signal == "STOP_ADC":
                print("Received STOP_ADC signal")
                data_acquisition.stop()  # Stop the data acquisition
                fraction_collector.stop()
                fraction_collector.stop_fraction_collector()
                error_detection.stop()
                #pump_a.stop()
            elif signal == "PAUSE_ADC":
                print("Received PAUSE_ADC signal")
                data_acquisition.pause()  # Pause the data acquisition
                fraction_collector.pause()
                fraction_collector.pause_fraction_collector()
                #pump_a.pause()
            elif signal == "RESUME_ADC":
                print("Received RESUME_ADC signal")
                data_acquisition.resume()  # Resume the data acquisition
                fraction_collector.resume()
                fraction_collector.resume_fraction_collector()
                #pump_a.resume()
            elif signal == "TOGGLE_LED":
                print("Received TOGGLE_LED signal")
                current_value = gpio_monitor.get_value(17)
                new_value = Value.ACTIVE if current_value == Value.INACTIVE else Value.INACTIVE
                gpio_monitor.set_value(17, new_value)
                print(f"LED toggled to {'ON' if new_value == Value.ACTIVE else 'OFF'}")     
             # Monitor operable_out status
            fraction_collector.monitor_error()   

    except socket.error as e:
        print(f"Socket error: {e}")
    finally:
        sock.close()
        data_acquisition.stop()  # Ensure data acquisition stops
        fraction_collector.stop()
        fraction_collector.stop_fraction_collector()
        error_detection.stop()
        #pump_a.stop()
        #valve_control.stop()
