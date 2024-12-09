import time
import ADS1256
import spidev
import numpy as np
import os

# Set the desired sampling frequency (Hz)
f_s = 400  # Adjust to the desired frequency (e.g., 600 Hz)
buffer_size = f_s * 10  # Record 10 seconds of data (adjust as needed)
data_file = '/home/patricia/ecg_data.csv'

# Pin configuration
SENSE = 2
SETUP_SPI = True

# Initialize ADC
ADC = ADS1256.ADS1256()
ADC.ADS1256_init()
ADC.ADS1256_ConfigADC(ADS1256.ADS1256_GAIN_E['ADS1256_GAIN_1'], ADS1256.ADS1256_DRATE_E['ADS1256_500SPS'])
ADC.ADS1256_SetChannal(SENSE)

# if SETUP_SPI:
#     SPI = spidev.SpiDev(0, 0)
#     SPI.mode = 0b01
#     SPI.max_speed_hz = 3000000

# Create a file to save data
if not os.path.exists(data_file):
    with open(data_file, 'w') as f:
        f.write("time,ecg_signal\n")  # Column headers

# Record data
try:
    print(f"Recording ECG data at {f_s} Hz...")
    start_time = time.time()
    while True:
        # Get current time for timestamp
        current_time = time.time() - start_time

        # Read ECG data from the ADC
        sense = ADC.ADS1256_GetChannalValue(SENSE) * 5.0 / 0x7fffff

        # Save to file (CSV format)
        with open(data_file, 'a') as f:
            f.write(f"{current_time},{sense}\n")

        # Calculate the elapsed time for this iteration
        elapsed_time = time.time() - current_time

        # Sleep to maintain the target sampling frequency
        time_to_sleep = (1 / f_s) - elapsed_time
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)

except KeyboardInterrupt:
    print("Data recording stopped.")
