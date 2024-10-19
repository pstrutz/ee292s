#!/Users/s/bin/python3
# Ege, 11/11/2024

import sys
import math as m
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


# Read sensor values from text file
def extract_first_world_values_from_file(file_path):
    """
    This function reads a text file and extracts the first element of each "world" entry.
    
    Args:
    - file_path: Path to the text file.

    Returns:
    - A list of first elements from the "world" entries.
    """
    world_values = []
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    for line in lines:
        if line.startswith("a:"):
            # Extract the values in the 'world' line and convert them to a tuple
            world_value = line.split(": (")[1].split(",")[0]
            # Append the first element of the world tuple to the result list
            world_values.append(float(world_value))
    
    return world_values

# Kalman Filter initialization
kf = KalmanFilter(dim_x=3, dim_z=1)  # state vector is [position, velocity, acceleration]

dt = 0.1
# State transition matrix
kf.F = np.array([[1., dt, (1/2)*dt**2], 
                 [0., 1., dt],
                 [0., 0., 1.]])

# Measurement matrix (we only observe acceleration)
kf.H = np.array([[0., 0., 1.]])

# Initial state: guess position, velocity, accel starts at 0
kf.x = np.array([0., 0., 0.])

# Initial covariance matrix (high uncertainty)
kf.P = np.array([[1e2, 0., 0.],
                 [0., 1e2, 0.],
                 [0., 0., 1e2]])

# Process noise covariance matrix (assuming very stable process)
# kf.Q = np.array([[0., 0., 0.],
#                  [0., 0., 0.],
#                  [0., 0., 0.]])
kf.Q = Q_discrete_white_noise(dim=3, dt=0.1, var=0.0001)
#kf.Q = np.eye(3) * 0.0001
# Constant for Earth gravity
GEE_TO_METERS = 9.81  # 1 gee = 9.81 m/s²
meas_var = GEE_TO_METERS**2*(0.01)**2 # from allan deviation average at sample size = 1

# Measurement noise (variance in the accel measurements)
kf.R = np.array([[meas_var]])

# Simulation arrays to store results
p_estimates = []
v_estimates = []
a_estimates = []
raw_measurements = extract_first_world_values_from_file("/Users/patriciastrutz/Library/CloudStorage/OneDrive-Stanford/Stanford/24-25a Fall/EE292S/ee292s/lab1/50cm_a.txt")
print(raw_measurements)
kalman_gains = []

taxis = np.array(range(len(raw_measurements)))
print(len(taxis))
print(len(raw_measurements))
# Run the simulation
for t in taxis:
    if t==0: continue
    # Feed measurement
    z = raw_measurements[t]
    u = z-raw_measurements[t-1]
    # Kalman filter prediction and update steps
    kf.predict(u=z)
    kf.update(z)
    
    # Store results
    p_estimates.append(kf.x[0])  # Estimated position
    v_estimates.append(kf.x[1])  # Estimated velocity
    a_estimates.append(kf.x[2])  # Estimated acceleration
    #sensor_measurements.append(z)  # Noisy sensor reading for accel
    kalman_gains.append(kf.K)  # Kalman gain

# Plot Position, Velocity, Acceleration in subplots (Kalman Filter estimates)
fig1, axs1 = plt.subplots(3, 1, figsize=(8, 10))
fig1.suptitle('Kalman Filter Estimates')

# Position
axs1[0].plot(taxis[1:]/10, p_estimates, label='KF position estimate')
axs1[0].set_ylabel('Position (m)')
axs1[0].grid()
axs1[0].legend()

# Velocity
axs1[1].plot(taxis[1:]/10, v_estimates, label='KF velocity estimate')
axs1[1].set_ylabel('Velocity (m/s)')
axs1[1].grid()
axs1[1].legend()

# Acceleration
axs1[2].plot(taxis[1:]/10, a_estimates, label='KF accel estimate')
axs1[2].set_xlabel('Time (seconds)')
axs1[2].set_ylabel('Accel (m/s^2)')
axs1[2].grid()
axs1[2].legend()

plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout and leave space for the title


# Plot Kalman Gains in subplots
fig2, axs2 = plt.subplots(3, 1, figsize=(8, 10))
fig2.suptitle('Kalman Gains')

# Ensure the shape of kalman_gains is compatible
kalman_gains_array = np.array(kalman_gains)

# Gain for Position
axs2[0].plot(taxis[1:]/10, kalman_gains_array[:, 0, :], label='Kalman Gain Position')
axs2[0].set_ylabel('Position Gain')
axs2[0].grid()

# Gain for Velocity
axs2[1].plot(taxis[1:]/10, kalman_gains_array[:, 1, :], label='Kalman Gain Velocity')
axs2[1].set_ylabel('Velocity Gain')
axs2[1].grid()

# Gain for Acceleration
axs2[2].plot(taxis[1:]/10, kalman_gains_array[:, 2, :], label='Kalman Gain Acceleration')
axs2[2].set_xlabel('Time (seconds)')
axs2[2].set_ylabel('Acceleration Gain')
axs2[2].grid()

plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout and leave space for the title

# Plot raw measurements (leave this separate)
plt.figure(3)
plt.plot(taxis/10, raw_measurements, label='Raw measurements')
plt.xlabel('Time (seconds)')
plt.ylabel('World X Acceleration')
plt.title("Sensor Measurement X Acceleration")
plt.grid()
plt.legend()

plt.show()