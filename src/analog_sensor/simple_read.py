#!/usr/bin/env python3 
import time
import Adafruit_ADS1x15

# Initialize ADS1015
adc = Adafruit_ADS1x15.ADS1015()

# Sensor on A0
channel = 0

# Set the gain (±4.096V FSR)
gain = 1

def convert_ads1015_to_arduino(ads1015_value):
    """Convert ADS1015 (12-bit, 3.3V) readings to Arduino scale (10-bit, 5V)."""
    return (ads1015_value / 4) * (5 / 3.3)

while True:
    # Read raw ADC value
    raw_value = adc.read_adc(channel, gain=gain)

    # Convert to Arduino equivalent value
    corrected_value = convert_ads1015_to_arduino(raw_value)

    print(f"Raw ADC Value: {raw_value} | Corrected (Arduino Scale): {corrected_value:.2f}")

    time.sleep(1)
