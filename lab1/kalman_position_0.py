#!/Users/s/bin/python3
# Ege, 11/11/2024

import sys
import math as m
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter

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
        if line.startswith("world:"):
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
                 [0., 0., 1e4]])

# Process noise covariance matrix (assuming very stable process)
kf.Q = np.array([[0., 0., 0.],
                 [0., 0., 0.],
                 [0., 0., 0.]])

# Constant for Earth gravity
GEE_TO_METERS = 9.81  # 1 gee = 9.81 m/s²
meas_var = GEE_TO_METERS**2*(0.03)**2 # from allan deviation average at sample size = 1
print(meas_var)

# Measurement noise (variance in the accel measurements)
kf.R = np.array([[meas_var]])

# Simulation arrays to store results
p_estimates = []
v_estimates = []
a_estimates = []
sensor_measurements = extract_first_world_values_from_file("/Users/egeturan/Documents/Sensing/SmartphoneSensors292S/ee292s/lab1/stationary.txt")
print(sensor_measurements)
kalman_gains = []

taxis = np.array(range(len(sensor_measurements)))
# Run the simulation
for t in taxis:
    # Feed measurement
    z = sensor_measurements[t]
    
    # Kalman filter prediction and update steps
    kf.predict()
    kf.update(z)
    
    # Store results
    p_estimates.append(kf.x[0])  # Estimated position
    v_estimates.append(kf.x[1])  # Estimated velocity
    a_estimates.append(kf.x[2])  # Estimated acceleration
    sensor_measurements.append(z)  # Noisy sensor reading for accel
    kalman_gains.append(kf.K)  # Kalman gain


# Plotting results
plt.figure()
plt.plot(taxis/10, p_estimates, label='KF position estimate')
plt.xlabel('Time (seconds)')
plt.ylabel('Position (m)')
plt.title('Kalman Filter Position Estimate')
plt.grid()
plt.legend()

# Plotting results
plt.figure()
plt.plot(taxis/10, v_estimates, label='KF velocity estimate')
plt.xlabel('Time (seconds)')
plt.ylabel('velocity (m/s)')
plt.title('Kalman Filter velocity Estimate')
plt.grid()
plt.legend()

# Plotting results
plt.figure()
plt.plot(taxis/10, a_estimates, label='KF accel estimate')
plt.xlabel('Time (seconds)')
plt.ylabel('Accel (m/s^2)')
plt.title('Kalman Filter Accel Estimate')
plt.grid()
plt.legend()

plt.show()
