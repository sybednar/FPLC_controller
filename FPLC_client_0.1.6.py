
#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
>>>>>>>>>>FPLC_client_0.1.6 (FPLC_controller) 
Functions:
1) ADS1115 ADC data acquisition
2) communicate with FPLC_server_GUI_0.3.13
3) GPIO communication 042425
'''

import socket
import threading
import time
import Adafruit_ADS1x15
import gpiod
from gpiod.line import Direction, Value, Bias
import os




# Data Acquisition Class
class DataAcquisition:
    def __init__(self, adc, gain, sampling_rate, sock):
        self.adc = adc
        self.gain = gain
        self.sampling_rate = sampling_rate
        self.sock = sock
        self.pause_event = threading.Event()
        self.stop_event = threading.Event()

    def start(self):
        self.acquisition_thread = threading.Thread(target=self.data_acquisition)
        self.acquisition_thread.start()

    def data_acquisition(self):
        start_time = time.time()
        while not self.stop_event.is_set():
            self.pause_event.wait()
            try:
                value1 = self.adc.read_adc(0, gain=self.gain)
                value2 = self.adc.read_adc(1, gain=self.gain)
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
            time.sleep(1 / self.sampling_rate)

        def stop(self):
            self.stop_event.set()
            self.pause_event.set()
            self.acquisition_thread.join()

        def pause(self):
            self.pause_event.clear()

        def resume(self):
            self.pause_event.set()

# Pump Control Class
class PumpControl:
    def __init__(self, gpio_chip, gpio_line):
        self.request = gpiod.request_lines(
            f"/dev/{gpio_chip}",
            consumer="pump_control",
            config={
                gpio_line: gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE
                )
            }
        )
        self.stop_event = threading.Event()

    def start(self):
        self.control_thread = threading.Thread(target=self.control_pump)
        self.control_thread.start()

    def control_pump(self):
        while not self.stop_event.is_set():
            # Add pump control logic here
            pass

    def stop(self):
        self.stop_event.set()
        self.control_thread.join()

# Valve Control Class
class ValveControl:
    def __init__(self, gpio_chip, gpio_line):
        self.request = gpiod.request_lines(
            f"/dev/{gpio_chip}",
            consumer="valve_control",
            config={
                gpio_line: gpiod.LineSettings(
                    direction=Direction.OUTPUT,
                    output_value=Value.INACTIVE
                )
            }
        )
        self.stop_event = threading.Event()

    def start(self):
        self.control_thread = threading.Thread(target=self.control_valve)
        self.control_thread.start()

    def control_valve(self):
        while not self.stop_event.is_set():
            # Add valve control logic here
            pass

    def stop(self):
        self.stop_event.set()
        self.control_thread.join()

# Main Script
if __name__ == '__main__':
    adc = Adafruit_ADS1x15.ADS1115(busnum=1)
    GAIN = 16
    sampling_rate = 10
    server_ip = '128.104.117.228'
    server_port = 5000

  
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((server_ip, server_port))
            print("Connected to server")
            break
        except socket.error as e:
            print(f"Socket error: {e}. Retrying...")
            time.sleep(2)

    data_acquisition = DataAcquisition(adc, GAIN, sampling_rate, sock)
    pump_control = PumpControl('gpiochip0', 5)
    valve_control = ValveControl('gpiochip0', 6)

    data_acquisition.start()
    pump_control.start()
    valve_control.start()

    try:
        while True:
            signal = sock.recv(1024).decode('utf-8')
            if signal == "START_ADC":
                data_acquisition.resume()
            elif signal == "STOP_ADC":
                data_acquisition.stop()
                break
            elif signal == "PAUSE_ADC":
                data_acquisition.pause()
            elif signal == "RESUME_ADC":
                data_acquisition.resume()
    except socket.error as e:
        print(f"Socket error: {e}")
    finally:
        sock.close()
        data_acquisition.stop()
        pump_control.stop()
        valve_control.stop()
