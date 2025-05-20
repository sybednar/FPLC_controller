#!/usr/bin/env python
# -*- coding: utf-8 -*-

#>>>>>>>>>>FPLC_client (FPLC_controller)
#Functions:
#1) ADS1115 ADC data acquisition

import socket
import Adafruit_ADS1x15
import time

adc = Adafruit_ADS1x15.ADS1115(busnum=1)
GAIN = 16  # Gain setting for 0-100mV signal
sampling_rate = 10  # 10Hz

server_ip = '128.104.117.228'  # Replace with your server's IP address
server_port = 5000

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, server_port))
except socket.error as e:
    print(f"Socket error: {e}")
    exit(1)

start_time = time.time()

try:
    while True:
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
        
        # Debugging prints
        print(f"Elapsed Time: {elapsed_time:.2f} s, Chan1: {value1}, Chan2: {value2}")
        
        time.sleep(1 / sampling_rate)
finally:
    sock.close()
