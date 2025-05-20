#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
>>>>>>>>>>FPLC_client_0.1.8 (FPLC_controller) 
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



class DataAcquisition:
    def __init__(self, adc, gain, sampling_rate, sock):
        self.adc = adc
        self.gain = gain
        self.sampling_rate = sampling_rate
        self.sock = sock
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
            print(f"Elapsed Time: {elapsed_time:.2f} s, Chan1: {value1}, Chan2: {value2}")
            time.sleep(1 / self.sampling_rate)

    def stop(self):
        self.stop_event.set()
        self.pause_event.set()  # Ensure the thread can exit
        self.acquisition_thread.join()

    def pause(self):
        self.pause_event.clear()

    def resume(self):
        self.pause_event.set()





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

    data_acquisition = DataAcquisition(adc, GAIN, sampling_rate, sock)
    #pump_control = PumpControl('gpiochip0', 5)
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
    except socket.error as e:
        print(f"Socket error: {e}")
    finally:
        sock.close()
        data_acquisition.stop()  # Ensure data acquisition stops
        #pump_control.stop()
        #valve_control.stop()
