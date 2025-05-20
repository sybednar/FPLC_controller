#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
>>>>>>>>>>FPLC_client_0.1.9.5.2 (FPLC_controller) 
Functions:
1) ADS1115 ADC data acquisition
2) communicate with FPLC_server_GUI_0.3.14
3) Threading 
4) GPIO communication 042725
5) Added Fraction Collector signal handling 050125
'''

import socket
import threading
import time
import Adafruit_ADS1x15
import gpiod
from gpiod.line import Direction, Value, Bias



class DataAcquisition:
    def __init__(self, adc, gain, sampling_rate, sock, gpio_monitor):
        self.adc = adc
        self.gain = gain
        self.sampling_rate = sampling_rate
        self.sock = sock
        self.gpio_monitor = gpio_monitor
        self.pause_event = threading.Event()
        self.stop_event = threading.Event()
        self.acquisition_thread = None

    def start(self):
        if self.acquisition_thread is None or not self.acquisition_thread.is_alive():
            self.acquisition_thread = threading.Thread(target=self.data_acquisition)
            self.acquisition_thread.start()

    def data_acquisition(self):
        start_time = time.time()
        previous_Frac_Mark_value = 0.0
        while not self.stop_event.is_set():
            self.pause_event.wait()
            try:
                value1 = self.adc.read_adc(0, gain=self.gain)
                value2 = self.adc.read_adc(1, gain=self.gain)
                gpio12_status = self.gpio_monitor.monitor_input(12)
            except Exception as e:
                print(f"ADC read error: {e}")
                continue
            elapsed_time = time.time() - start_time
            if (gpio12_status == 'LOW' and previous_Frac_Mark_value == 0.0):
                Frac_Mark = 1.0
                previous_Frac_Mark_value = Frac_Mark
            else:
                Frac_Mark = 0.0
                previous_Frac_Mark_value = Frac_Mark
                
            data = f"{value1},{value2},{elapsed_time},{Frac_Mark}"
            try:
                self.sock.sendall(data.encode('utf-8'))
            except socket.error as e:
                print(f"Socket send error: {e}")
                break
            print(f"Elapsed Time: {elapsed_time:.2f} s, Chan1: {value1}, Chan2: {value2}, GPIO12: {gpio12_status}")
            time.sleep(1 / self.sampling_rate)

    def stop(self):
        self.stop_event.set()
        self.pause_event.set()  # Ensure the thread can exit
        if self.acquisition_thread is not None and self.acquisition_thread.is_alive():
            self.acquisition_thread.join()
        self.acquisition_thread = None

    def pause(self):
        self.pause_event.clear()

    def resume(self):
        self.pause_event.set()

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





class FractionCollector:
   def __init__(self, gpio_control):
       self.gpio_control = gpio_control
       self.pins = {
            'operable_in': 5,
            'start_stop': 6,
            'pause_resume': 13,
            'operable_out': 16,
            'error': 26
       }

   def set_pin(self, pin_name, value):
       self.gpio_control.set_value(self.pins[pin_name], value)

   def get_pin_status(self, pin_name):
       return self.gpio_control.monitor_input(self.pins[pin_name])

   def start(self):
       self.set_pin('start_stop', GPIOControl.LOW)
       self.print_status('operable_out')

   def stop(self):
       if self.get_pin_status('operable_out') == "LOW":
            self.set_pin('start_stop', GPIOControl.HIGH)
       print("Fraction collector stopped")
       self.print_status('operable_out')

   def pause(self):
       if self.get_pin_status('operable_out') == "LOW" and self.get_pin_status('start_stop') == "LOW":
            self.set_pin('pause_resume', GPIOControl.LOW)
            print("Fraction collector paused")

   def resume(self):
       if (self.get_pin_status('operable_out') == "LOW" and 
            self.get_pin_status('start_stop') == "LOW" and 
            self.get_pin_status('pause_resume') == "LOW"):
            self.set_pin('pause_resume', GPIOControl.HIGH)
            print("Fraction collector resumed")

   def print_status(self, pin_name):
       status = self.get_pin_status(pin_name)
       print(f"GPIO{self.pins[pin_name]} status: {status}")
   
   def monitor_error(self): 
       if self.get_pin_status('error') == "LOW":
           print("Fraction Collector ERROR has occurred")
           


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
            print(f"Socket error: {e}. Retrying...")
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
    fraction_collector = FractionCollector(gpio_monitor)
    #pump_a = PumpA(gpio_monitor)

    try:
        while True:
            signal = sock.recv(1024).decode('utf-8')
            if signal == "START_ADC":
                print("Received START_ADC signal")
                data_acquisition.stop_event.clear()  # Ensure the stop event is cleared
                data_acquisition.resume()  # Ensure the pause event is set to resume
                data_acquisition.start()
                fraction_collector.start()
                #pump_a.start()
            elif signal == "STOP_ADC":
                print("Received STOP_ADC signal")
                data_acquisition.stop()  # Stop the data acquisition
                fraction_collector.stop()
                #pump_a.stop()
            elif signal == "PAUSE_ADC":
                print("Received PAUSE_ADC signal")
                data_acquisition.pause()  # Pause the data acquisition
                fraction_collector.pause()
                #pump_a.pause()
            elif signal == "RESUME_ADC":
                print("Received RESUME_ADC signal")
                data_acquisition.resume()  # Resume the data acquisition
                fraction_collector.resume()
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
        #fraction_collector.stop()
        #pump_a.stop()
        #pump_control.stop()
        #valve_control.stop()
