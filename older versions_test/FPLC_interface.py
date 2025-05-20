import socket
import Adafruit_ADS1x15
import time

adc = Adafruit_ADS1x15.ADS1115(busnum=1)
GAIN = 1
sampling_rate = 10  # 10Hz

server_ip = '128.104.117.228'  # Replace with your server's IP address
server_port = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))

start_time = time.time()

while True:
    value1 = adc.read_adc(0, gain=GAIN)
    value2 = adc.read_adc(1, gain=GAIN)
    elapsed_time = time.time() - start_time
    data = f"{value1},{value2}"
    sock.sendall(data.encode('utf-8'))
    
    # Debugging prints
    print(f"Elapsed Time: {elapsed_time:.2f} s, Chan1: {value1}, Chan2: {value2}")
    
    time.sleep(1 / sampling_rate)
