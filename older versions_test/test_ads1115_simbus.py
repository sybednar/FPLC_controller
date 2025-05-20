import smbus2
import time

# ADS1115 I2C address
ADS1115_ADDRESS = 0x48

# Register addresses
ADS1115_CONVERSION_REG = 0x00
ADS1115_CONFIG_REG = 0x01

# Configuration register values
CONFIG_OS_SINGLE = 0x8000  # Start single conversion
CONFIG_MUX_SINGLE_0 = 0x4000  # Single-ended AIN0
CONFIG_PGA_4_096V = 0x0200  # +/- 4.096V range
CONFIG_MODE_SINGLE = 0x0100  # Single-shot mode
CONFIG_DR_128SPS = 0x0080  # 128 samples per second
CONFIG_CMODE_TRAD = 0x0000  # Traditional comparator
CONFIG_CPOL_ACTVLOW = 0x0000  # Active low
CONFIG_CLAT_NONLAT = 0x0000  # Non-latching
CONFIG_CQUE_NONE = 0x0003  # Disable comparator

# Combine configuration register values
config = (CONFIG_OS_SINGLE | CONFIG_MUX_SINGLE_0 | CONFIG_PGA_4_096V |
          CONFIG_MODE_SINGLE | CONFIG_DR_128SPS | CONFIG_CMODE_TRAD |
          CONFIG_CPOL_ACTVLOW | CONFIG_CLAT_NONLAT | CONFIG_CQUE_NONE)

# Initialize I2C bus
bus = smbus2.SMBus(1)

def read_voltage():
    # Write config register to start conversion
    bus.write_i2c_block_data(ADS1115_ADDRESS, ADS1115_CONFIG_REG, [(config >> 8) & 0xFF, config & 0xFF])
    time.sleep(0.1)  # Wait for conversion to complete

    # Read conversion result
    data = bus.read_i2c_block_data(ADS1115_ADDRESS, ADS1115_CONVERSION_REG, 2)
    raw_adc = (data[0] << 8) | data[1]

    # Convert raw ADC value to voltage
    voltage = raw_adc * 4.096 / 32768.0
    return voltage

while True:
    voltage = read_voltage()
    print(f"Voltage: {voltage:.4f} V")
    time.sleep(1)
