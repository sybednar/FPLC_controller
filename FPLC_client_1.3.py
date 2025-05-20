#!/usr/bin/env python
# -*- coding: utf-8 -*-

#>>>>>>>>>>FPLC_client_1.3 (FPLC_controller) 042125 11pm
#Functions:
#1) ADS1115 ADC data acquisition

import socket
import Adafruit_ADS1x15
import time
import threading

adc = Adafruit_ADS1x15.ADS1115(busnum=1)
GAIN = 16 # Gain setting for 0-100mV signal
sampling_rate = 10 # 10Hz
server_ip = '128.104.117.228' # Replace with your server's IP address
server_port = 5000

# Retry mechanism to connect to the server
while True:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((server_ip, server_port))
        print("Connected to server")
        break
    except socket.error as e:
        print(f"Socket error: {e}. Retrying...")
        time.sleep(2)

pause_event = threading.Event()
stop_event = threading.Event()

def data_acquisition():
    start_time = time.time()
    while not stop_event.is_set():
        pause_event.wait() # Wait until the event is set (resume)
        try:
            value1 = adc.read_adc(0, gain=GAIN)
            value2 = adc.read_adc(1, gain=GAIN)
        except Exception as e:
            print(f"ADC read error: {e}")
            continue

        elapsed_time = time.time() - start_time
        data = f"{value1},{value2},{elapsed_time}"
        try:
            sock.sendall(data.encode('utf-8'))
        except socket.error as e:
            print(f"Socket send error: {e}")
            break


        print(f"Elapsed Time: {elapsed_time:.2f} s, Chan1: {value1}, Chan2: {value2}")
        time.sleep(1 / sampling_rate)

try:
    while True:
        signal = sock.recv(1024).decode('utf-8')
        if signal == "START_ADC":
            print("Received START_ADC signal")
            stop_event.clear() # Ensure the stop event is cleared
            pause_event.set() # Ensure the pause event is set (resume)
            acquisition_thread = threading.Thread(target=data_acquisition)
            acquisition_thread.start()
        elif signal == "STOP_ADC":
            print("Received STOP_ADC signal")
            stop_event.set() # Set the event to stop the thread
            pause_event.set() # Ensure the pause event is set to allow the thread to exit
            acquisition_thread.join()
            break
        elif signal == "PAUSE_ADC":
            print("Received PAUSE_ADC signal")
            pause_event.clear() # Clear the event to pause the thread
        elif signal == "RESUME_ADC":
            print("Received RESUME_ADC signal")
            pause_event.set() # Set the event to resume the thread
except socket.error as e:
    print(f"Socket error: {e}")
finally:
    sock.close()