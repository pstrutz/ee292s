import ICM20948 as ICM
import time
import numpy as np

def calculate_tilt_from_accel(accel):
    """Calculate pitch and roll from accelerometer data."""
    pitch = np.arctan2(accel[1], np.sqrt(accel[0]**2 + accel[2]**2)) * (180.0 / np.pi)
    roll = np.arctan2(-accel[0], accel[2]) * (180.0 / np.pi)
    return pitch, roll

def calculate_tilt_from_gyro(gyro, dt):
    """Calculate pitch and roll from gyroscope data."""
    pitch_rate = gyro[0] * dt  # Gyro x-axis (pitch rate)
    roll_rate = gyro[1] * dt   # Gyro y-axis (roll rate)
    return pitch_rate, roll_rate

def calculate_combined_tilt(pitch, roll):
    """Calculate combined tilt from pitch and roll."""
    return np.sqrt(pitch**2 + roll**2)

if __name__ == '__main__':
    print("\nSense HAT Test Program ...\n")
    icm20948 = ICM.ICM20948()

    # Initialize time
    start_time = time.time()
    
    while True:
        # Read sensors
        icm20948.icm20948_Gyro_Accel_Read()
        icm20948.icm20948MagRead()
        icm20948.icm20948CalAvgValue()
        time.sleep(0.1)

        # Get current time
        current_time = time.time() - start_time  # Timer starts at 0
        dt = 0.1  # Fixed sleep time, can adjust for more accurate timing

        # Update AHRS
        icm20948.imuAHRSupdate(icm20948.motionVal[0] * 0.0175, icm20948.motionVal[1] * 0.0175,
                                icm20948.motionVal[2] * 0.0175, icm20948.motionVal[3],
                                icm20948.motionVal[4], icm20948.motionVal[5], 
                                icm20948.motionVal[6], icm20948.motionVal[7], 
                                icm20948.motionVal[8])

        # Get accelerometer and gyroscope readings
        accel_adc = [ICM.Accel[0], ICM.Accel[1], ICM.Accel[2]]
        gyro_adc = [ICM.Gyro[0], ICM.Gyro[1], ICM.Gyro[2]]
        accel = [accel_adc[i] * (2.0 / 2**15) for i in range(len(accel_adc))]     # g
        gyro = [gyro_adc[i] * (1000.0 / 2**15) for i in range(len(gyro_adc))]    # dps

        # Calculate tilt from accelerometer and gyroscope
        accel_pitch, accel_roll = calculate_tilt_from_accel(accel)
        gyro_pitch_rate, gyro_roll_rate = calculate_tilt_from_gyro(gyro, dt)

        # Calculate combined tilts
        combined_tilt_accel = calculate_combined_tilt(accel_pitch, accel_roll)
        combined_tilt_gyro = calculate_combined_tilt(gyro_pitch_rate, gyro_roll_rate)

        # Print the results
        print(f"{current_time:.2f} - Combined Tilt (Accel): {combined_tilt_accel:.2f}° - Combined Tilt (Gyro): {combined_tilt_gyro:.2f}°")
