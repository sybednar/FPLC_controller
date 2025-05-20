#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
>>>>>>>>>>FPLC_client_0.1.9 (FPLC_controller) 
Functions:
1) ADS1115 ADC data acquisition
2) communicate with FPLC_server_GUI_0.3.13
3) Threading 
3) GPIO communication 042525
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
        self.acquisition_thread = threading.Thread(target=self.data_acquisition)

    def start(self): 
        self.acquisition_thread.start()

    def data_acquisition(self):
        start_time = time.time()
        while not self.stop_event.is_set():
            self.pause_event.wait()
            try:
                value1 = self.adc.read_adc(0, gain=self.gain)
                value2 = self.adc.read_adc(1, gain=self.gain)
                gpio_value = self.gpio_monitor.get_value()
                gpio_status = "HIGH" if gpio_value == Value.ACTIVE else "LOW"
            except Exception as e:
                print(f"ADC read error: {e}")
                continue
            elapsed_time = time.time() - start_time
            data = f"{value1},{value2},{elapsed_time}"
            try:
                self.sock.sendall(data.encode('utf-8'))
            except socket.error as e:
                print(f"Socket send error: {e}")
                break
            print(f"Elapsed Time: {elapsed_time:.2f} s, Chan1: {value1}, Chan2: {value2}, GPIO6: {gpio_status}")
            time.sleep(1 / self.sampling_rate)

    def stop(self):
        self.stop_event.set()
        self.pause_event.set()  # Ensure the thread can exit
        self.acquisition_thread.join()

    def pause(self):
        self.pause_event.clear()

    def resume(self):
        self.pause_event.set()



class GPIOControl:
    def __init__(self, chip_name, line_num):
        self.chip_name = chip_name
        self.line_num = line_num
        self.request = None
        self.setup_gpio()
        
    def setup_gpio(self):
        self.request = gpiod.request_lines(
            f"/dev/{self.chip_name}",
            consumer="gpio_control",
            config={
                self.line_num: gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE
                )
            }
        )
    def set_value(self, value):
        self.request.set_value(self.line_num, value)
    
    def get_value(self):
        return self.request.get_value(self.line_num)
    
 

    def monitor_input(self, input_line_num):
        self.request = gpiod.request_lines(
            f"/dev/{self.chip_name}",
            consumer="gpio_control",
            config={
                input_line_num: gpiod.LineSettings(
                direction=Direction.INPUT,
                bias=Bias.PULL_UP
                )
            }
        )
        while True:
            value = self.request.get_value(input_line_num)
            if value == Value.ACTIVE:
                print(f"GPIO{input_line_num} is HIGH")
            else:
                print(f"GPIO{input_line_num} is LOW")
                
            time.sleep(0.1)


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

    gpio_monitor = GPIOControl('gpiochip0', 6)
    data_acquisition = DataAcquisition(adc, GAIN, sampling_rate, sock, gpio_monitor)
    gpio_control = GPIOControl('gpiochip0', 5)
    
    #valve_control = ValveControl('gpiochip0', 18)

    #data_acquisition.start()
    #pump_control.start()
    #valve_control.start()

    try:
        while True:
            signal = sock.recv(1024).decode('utf-8')
            if signal == "START_ADC":
                print("Received START_ADC signal")
                data_acquisition.stop_event.clear()  # Ensure the stop event is cleared
                data_acquisition.resume()  # Ensure the pause event is set to resume
                if not data_acquisition.acquisition_thread.is_alive():
                    data_acquisition.start()
            elif signal == "STOP_ADC":
                print("Received STOP_ADC signal")
                data_acquisition.stop()  # Stop the data acquisition
                break
            elif signal == "PAUSE_ADC":
                print("Received PAUSE_ADC signal")
                data_acquisition.pause()  # Pause the data acquisition
            elif signal == "RESUME_ADC":
                print("Received RESUME_ADC signal")
                data_acquisition.resume()  # Resume the data acquisition
            elif signal == "TOGGLE_LED":
                print("Received TOGGLE_LED signal")
                current_value = gpio_control.request.get_value(gpio_control.line_num)
                new_value = Value.ACTIVE if current_value == Value.INACTIVE else Value.INACTIVE
                gpio_control.set_value(new_value)
                print(f"LED toggled to {'ON' if new_value == Value.ACTIVE else 'OFF'}")     
                

    except socket.error as e:
        print(f"Socket error: {e}")
    finally:
        sock.close()
        data_acquisition.stop()  # Ensure data acquisition stops
        #pump_control.stop()
        #valve_control.stop()
