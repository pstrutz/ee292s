import ICM20948 as ICM
import time
import numpy as np
from datetime import datetime

import numpy as np

# Constants
g = 9.81  # Gravity (m/s^2)


def calibrate_accelerometer(icm20948, calibration_time=5):
    print("Calibrating accelerometer...")
    samples = 100
    bias = np.zeros(3)
    
    for _ in range(samples):
        icm20948.icm20948_Gyro_Accel_Read()
        accel_adc = [ICM.Accel[0], ICM.Accel[1], ICM.Accel[2]]
        accel = [accel_adc[i] * (2.0 / 2**15) for i in range(len(accel_adc))]  # g
        
        bias += np.array(accel)
        time.sleep(calibration_time / samples)  # Sleep to evenly distribute samples

    # Calculate the average bias
    bias /= samples
    print(f"Calibration complete. Bias: {bias}")
    return bias

if __name__ == '__main__':
    print("\nSense HAT Test Program ...\n")
    icm20948 = ICM.ICM20948()
    
    # Initialize current angles for gyro integration
    current_angles = (0.0, 0.0, 0.0)  # roll, pitch, yaw (starts at 0 degrees)

    # Calibration of the accelerometer
    bias = calibrate_accelerometer(icm20948)
    print("Bias is: ", bias)
    # Record the start time
    start_time = time.time()

    # Target frequency and interval
    target_frequency = 100  # Hz
    target_interval = 1.0 / target_frequency  # Interval in seconds (0.1 seconds)

    # Create a file with the desired timestamp format
    timestamp = datetime.now().strftime("calibrated_%Y_%m_%d_%H_%M_%S")
    filename = timestamp
    with open(filename, 'w') as file:
        while True:
            loop_start_time = time.time()  # Record the time when the loop starts

            icm20948.icm20948_Gyro_Accel_Read()
            accel_adc = [ICM.Accel[0], ICM.Accel[1], ICM.Accel[2]]
            gyro_adc = [ICM.Gyro[0], ICM.Gyro[1], ICM.Gyro[2]]
            accel = [accel_adc[i] * (2.0 / 2**15) for i in range(len(accel_adc))]  # g
            gyro = [gyro_adc[i] * (1000.0 / 2**15) for i in range(len(gyro_adc))]  # dps (degrees per second)

            a = np.array(accel) - bias
            g = np.array(gyro) 
        
            # Get elapsed time since start
            elapsed_time = time.time() - start_time

            # Prepare the data to print and write to the file
            data = (
                f"time: {elapsed_time:.5f}\n"
                f"a: ({a[0]:.5f}, {a[1]:.5f}, {a[2]:.5f})\n"        # gee
                f"g: ({g[0]:.5f}, {g[1]:.5f}, {g[2]:.5f})\n"        # dps (degrees per second)
            )

            # Print the data to the console
            # print(data)

            # Write the data to the file
            file.write(data)

            # Calculate loop duration
            loop_duration = time.time() - loop_start_time

            # Sleep for the remaining time to maintain the required Hz rate
            if loop_duration < target_interval:
                time.sleep(target_interval - loop_duration)