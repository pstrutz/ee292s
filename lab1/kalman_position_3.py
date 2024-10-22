#!/Users/s/bin/python3
# Ege, 11/11/2024

import sys
import math as m
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.spatial.transform import Rotation as R

def normalize(v):
    """ Normalize a vector """
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

# Read sensor values from text file
def extract_values_from_file(file_path):
    """
    This function reads a text file and extracts the first element of each "world" entry.
    
    Args:
    - file_path: Path to the text file.

    Returns:
    - A list of first elements from the "world" entries.
    """
    a_values = []
    g_values = []
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    for line in lines:
        a_value = np.zeros((1,3))
        g_value = np.zeros((1,3))
        if line.startswith("a:"):
            # Extract the values in the 'world' line and convert them to a tuple
            a_value0 = line.split(": (")[1].split(",")[0]
            print(line.split(": (")[1].split(",")[1])
            a_value1 = line.split(": (")[1].split(",")[1]
            a_value2 = line.split(": (")[1].split(",")[2].split(")")[0]
            # Append the first element of the world tuple to the result list
            a_values.append([float(a_value0), float(a_value1), float(a_value2)])
        if line.startswith("g:"):
            # Extract the values in the 'world' line and convert them to a tuple
            g_value0 = line.split(": (")[1].split(",")[0]
            g_value1 = line.split(": (")[1].split(",")[1]
            g_value2 = line.split(": (")[1].split(",")[2].split(")")[0]
            # Append the first element of the world tuple to the result list
            g_values.append([float(g_value0), float(g_value1), float(g_value2)])
    
    return a_values, g_values


def find_world_acceleration(acc_data, gyro_data, dt, g=9.81, alpha_roll=0.98, alpha_pitch=0.99, alpha_yaw=0):
    """
    Estimates orientation using a complementary filter with different alphas for roll, pitch, and yaw.
    Subtracts gravity from accelerometer readings.
    
    acc_data: (N, 3) accelerometer readings in sensor frame (N samples, x, y, z)
    gyro_data: (N, 3) gyroscope readings in radians per second (x, y, z)
    dt: time step between readings (seconds)
    alpha_roll: blending factor for roll
    alpha_pitch: blending factor for pitch
    alpha_yaw: blending factor for yaw
    
    Returns:
        acc_world_frame: (N, 3) accelerometer data in world frame with gravity subtracted
    """
    num_samples = acc_data.shape[0]
    acc_world_frame = np.zeros_like(acc_data)
    
    # Initial orientation (assuming aligned with world axes initially)
    orientation = R.identity()

    for i in range(num_samples):
        # Update orientation from gyroscope data (integrate angular velocity)
        gyro_angle = gyro_data[i] * dt
        delta_orientation = R.from_rotvec(gyro_angle)
        gyro_orientation = orientation * delta_orientation

        # Estimate tilt from accelerometer (world frame assumption is z-axis aligned with gravity)
        acc_tilt = acc_data[i]
        acc_norm = normalize(acc_tilt)
        
        # Estimate orientation based on accelerometer (pitch and roll from gravity direction)
        acc_orientation = R.from_euler('xyz', [np.arctan2(acc_norm[1], acc_norm[2]),  # Roll
                                               np.arctan2(-acc_norm[0], np.sqrt(acc_norm[1]**2 + acc_norm[2]**2)),  # Pitch
                                               0], degrees=False)  # Yaw is not estimated from accelerometer

        # Extract the Euler angles for both orientations (gyro and accelerometer)
        gyro_euler = gyro_orientation.as_euler('xyz', degrees=False)
        acc_euler = acc_orientation.as_euler('xyz', degrees=False)

        # Apply complementary filter with different alphas for each axis
        filtered_euler = np.zeros(3)
        filtered_euler[0] = alpha_roll * gyro_euler[0] + (1 - alpha_roll) * acc_euler[0]  # Roll
        filtered_euler[1] = alpha_pitch * gyro_euler[1] + (1 - alpha_pitch) * acc_euler[1]  # Pitch
        filtered_euler[2] = alpha_yaw * gyro_euler[2] + (1 - alpha_yaw) * acc_euler[2] # Yaw (gyroscope only)

        # Convert filtered Euler angles back to a rotation matrix
        orientation = R.from_euler('xyz', filtered_euler, degrees=False)

        # Transform accelerometer data to world frame
        acc_world_frame[i] = orientation.apply(acc_data[i])

        # Subtract gravity (assumed in world z-axis direction)
        acc_world_frame[i][2] -= g*1.014

    return acc_world_frame


# Simulation arrays to store results
p_estimates = []
v_estimates = []
a_estimates = []
kalman_gains = []
dt = 0.1
GEE_TO_METERS = 9.81  # 1 gee = 9.81 m/s²
meas_var = GEE_TO_METERS**2*(0.01)**2 # from allan deviation average at sample size = 1

raw_measurements_accel, raw_measurements_gyro = extract_values_from_file("/Users/patriciastrutz/Library/CloudStorage/OneDrive-Stanford/Stanford/24-25a Fall/EE292S/ee292s/lab1/30cm.txt")
acc_world_frame = find_world_acceleration(np.array(raw_measurements_accel), np.array(raw_measurements_gyro), 0.1)

kf = KalmanFilter(dim_x=2, dim_z=1)  # 2 state variables (position, velocity), 1 measurement (acceleration)
# State transition matrix (position and velocity update)
kf.F = np.array([[1, dt], 
                [0, 1]])
      
# Measurement function (we are measuring only acceleration)
kf.H = np.array([[0, 0]])
        
# Initial state (position, velocity)
kf.x = np.zeros(2)
        
# Covariance matrix
kf.P = np.eye(2)  # Initial covariance
        
process_var = meas_var
# Process noise covariance (model uncertainty)
kf.Q = process_var * np.array([[0.5 * dt**2, 0],
                                [0, dt]])      
# Measurement noise covariance (sensor noise)
kf.R = np.array([[meas_var]])
        
# Control input matrix (relating acceleration to velocity)
kf.B = np.array([[0.5 * dt**2], [dt]])

num_samples = acc_world_frame.shape[0]

taxis = np.array(range(num_samples))
# Example loop for Kalman filter using accelerometer data
for t in range(num_samples):
    acc_x = acc_world_frame[t][0]  # Use the X-axis acceleration in world frame

    # Predict step (use accelerometer data)
    kf.predict(u=acc_x)

    # Get the estimated state (position, velocity)
    estimated_position, estimated_velocity = kf.x
    p_estimates.append(estimated_position)
    v_estimates.append(estimated_velocity)
    kalman_gains.append(kf.K)

    #print(f"Estimated Position: {estimated_position}, Estimated Velocity: {estimated_velocity}")



# Plot Position, Velocity, Acceleration in subplots (Kalman Filter estimates)
fig1, axs1 = plt.subplots(2, 1, figsize=(8, 10))
fig1.suptitle('Kalman Filter Estimates')

# Position
p_estimates = np.array(p_estimates)
axs1[0].plot(taxis/10, p_estimates/10, label='KF position estimate')
axs1[0].set_ylabel('Position (m)')
axs1[0].grid()
axs1[0].legend()

# Velocity
v_estimates = np.array(v_estimates)
axs1[1].plot(taxis/10, v_estimates/10, label='KF velocity estimate')
axs1[1].set_ylabel('Velocity (m/s)')
axs1[1].grid()
axs1[1].legend()

plt.figure(2) 
plt.plot(acc_world_frame[:, 0])

plt.legend()

plt.show()
# # Plot Kalman Gains in subplots
# fig2, axs2 = plt.subplots(2, 1, figsize=(8, 10))
# fig2.suptitle('Kalman Gains')

# # Ensure the shape of kalman_gains is compatible
# kalman_gains_array = np.array(kalman_gains)

# # # Gain for Position
# # axs2[0].plot(taxis/10, kalman_gains_array[:, 0, :], label='Kalman Gain Position')
# # axs2[0].set_ylabel('Position Gain')
# # axs2[0].grid()

# # # Gain for Velocity
# # axs2[1].plot(taxis/10, kalman_gains_array[:, 1, :], label='Kalman Gain Velocity')
# # axs2[1].set_ylabel('Velocity Gain')
# # axs2[1].grid()


# plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout and leave space for the title

