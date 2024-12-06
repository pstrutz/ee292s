import ICM20948 as ICM
import time
import numpy as np
from datetime import datetime

# Function to calculate roll, pitch, and yaw from accelerometer
def calculate_roll_pitch_yaw_accel(accel):
    ax, ay, az = accel
    roll = np.arctan2(-ay, np.sqrt(ax**2 + az**2)) * 180 / np.pi  # Roll angle in degrees
    pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2)) * 180 / np.pi  # Pitch angle in degrees
    yaw = np.arctan2(ay, ax) * 180 / np.pi  # Approximate Yaw angle in degrees (limited accuracy without magnetometer)
    return roll, pitch, yaw

# Function to integrate gyro data to estimate roll, pitch, and yaw
def integrate_gyro(gyro, dt, current_angles):
    roll, pitch, yaw = current_angles
    # Convert angular velocity from degrees/second to radians/second
    gyro_rad = np.radians(gyro)
    
    thresh = 0.00
    # Integrate angular velocities over time to update roll, pitch, and yaw
    # Check if each gyro value is greater than or equal to 0.5 radians per second
    if abs(gyro_rad[0]) >= thresh:  # Roll
        roll += gyro_rad[0] * dt * 180 / np.pi  # Integrate roll
    
    if abs(gyro_rad[1]) >= thresh:  # Pitch
        pitch += gyro_rad[1] * dt * 180 / np.pi  # Integrate pitch
    
    if abs(gyro_rad[2]) >= thresh:  # Yaw
        yaw += gyro_rad[2] * dt * 180 / np.pi    # Integrate yaw

    # Normalize angles to be within -180 to 180
    roll %= 360
    pitch %= 360
    yaw %= 360

    roll = roll if roll < 180 else roll - 360
    pitch = pitch if pitch < 180 else pitch - 360
    yaw = yaw if yaw < 180 else yaw - 360

    return roll, pitch, yaw


# Alpha vector for complementary filter: [0.98, 0.98, 0] (favor accelerometer for roll, pitch; fully trust gyro for yaw)
alpha = [0.98, 0.98, 0]  # Adjust alpha for each axis

if __name__ == '__main__':
    print("\nSense HAT Test Program ...\n")
    icm20948 = ICM.ICM20948()
    
    # Initialize current angles for gyro integration
    global current_angles
    current_angles = (0.0, 0.0, 0.0)  # roll, pitch, yaw (starts at 0 degrees)

    # Record the start time
    start_time = time.time()

    # Target frequency and interval
    target_frequency = 10  # Hz
    target_interval = 1.0 / target_frequency  # Interval in seconds (0.1 seconds)

    # Create a file with the desired timestamp format
    timestamp = datetime.now().strftime("angles_%Y_%m_%d_%H_%M_%S")
    filename = f"{timestamp}.txt"
    
    with open(filename, 'a') as file:
        while True:
            loop_start_time = time.time()  # Record the time when the loop starts

            icm20948.icm20948_Gyro_Accel_Read()

            accel_adc = [ICM.Accel[0], ICM.Accel[1], ICM.Accel[2]]
            gyro_adc = [ICM.Gyro[0], ICM.Gyro[1], ICM.Gyro[2]]
            accel = [accel_adc[i] * (2.0 / 2**15) for i in range(len(accel_adc))]     # g
            gyro = [gyro_adc[i] * (1000.0 / 2**15) for i in range(len(accel_adc))]    # dps (degrees per second)

            # Calculate roll, pitch, and yaw from accelerometer
            roll_acce, pitch_acce, yaw_acce = calculate_roll_pitch_yaw_accel(accel)
            
            # Integrate gyro data for roll, pitch, and yaw (dt = 0.1s)
            current_angles = integrate_gyro(gyro, dt=target_interval, current_angles=current_angles)

            # Combine accelerometer and gyro using custom alpha vector
            roll_filtered = alpha[0] * roll_acce + (1 - alpha[0]) * current_angles[0]
            pitch_filtered = alpha[1] * pitch_acce + (1 - alpha[1]) * current_angles[1]
            yaw_filtered = current_angles[2]  # Only use gyroscope for yaw (alpha[2] = 0)

            # Calculate net tilt in degrees
            net_tilt = np.sqrt(roll_filtered**2 + pitch_filtered**2)
            acce_tilt = np.sqrt(roll_acce**2 + pitch_acce**2)
            gyro_tilt = np.sqrt(current_angles[0]**2 + current_angles[1]**2)

            # Get elapsed time since start
            elapsed_time = time.time() - start_time

            # Prepare the data to print and write to the file
            data = (
                f"time: {elapsed_time:.2f}\n"
                f"acce: ({roll_acce:.2f}, {pitch_acce:.2f}, {yaw_acce:.2f})\n"
                f"gyro: ({current_angles[0]:.2f}, {current_angles[1]:.2f}, {current_angles[2]:.2f})\n"
                f"filtered: ({roll_filtered:.2f}, {pitch_filtered:.2f}, {yaw_filtered:.2f})\n"
                f"acce tilt: {acce_tilt:.2f}\n"  # Add acce tilt to output
                f"gyro tilt: {gyro_tilt:.2f}\n"  # Add gyro tilt to output
                f"filtered tilt: {net_tilt:.2f}\n\n"  # Add net tilt to output
            )

            # Print the data to the console
            print(data)

            # Write the data to the file
            file.write(data)

            # Calculate loop duration
            loop_duration = time.time() - loop_start_time

            # Sleep for the remaining time to maintain 10Hz rate
            if loop_duration < target_interval:
                time.sleep(target_interval - loop_duration)
