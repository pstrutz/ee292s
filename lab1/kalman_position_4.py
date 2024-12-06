#!/Users/s/bin/python3
# Ege, 11/11/2024

import sys
import math as m
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.spatial.transform import Rotation as R

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
    
    return a_values,g_values

# Simulation arrays to store results
p_estimates = []
v_estimates = []
a_estimates = []
kalman_gains = []
dt = 0.01
GEE_TO_METERS = 9.81  # 1 gee = 9.81 m/s²
meas_var = GEE_TO_METERS**2*(0.01)**2 # from allan deviation average at sample size = 1

raw_measurements_accel, raw_measurements_gyro = extract_values_from_file("/Users/egeturan/Documents/Sensing/SmartphoneSensors292S/ee292s/lab1/oct23/6ft_medium_round_straight")
raw_accel = 9.81*np.array(raw_measurements_accel)
raw_gyro = np.array(raw_measurements_gyro)

kf = KalmanFilter(dim_x=3, dim_z=1)  # 2 state variables (position, velocity), 1 measurement (acceleration)
# State transition matrix (position and velocity update)
kf.F = np.array([[1, dt, 0.5*dt**2], 
                [0, 1, dt],
     
               [0, 0, 1]])
      
# Measurement function (we are measuring only acceleration)
kf.H = np.array([[0, 0, 1]])
        
# Initial state (position, velocity, accel)
kf.x = np.zeros(3)
        
# Initial covariance matrix (high uncertainty)
kf.P = np.eye(3)*1e-2

        
# Process noise covariance matrix (assuming very stable process)
# kf.Q = np.array([[0., 0., 0.],
#                  [0., 0., 0.],
#                  [0., 0., 0.]])
# kf.Q = Q_discrete_white_noise(dim=3, dt=0.1, var=0.1)
kf.Q = np.eye(3)*0.1


# Measurement noise (variance in the accel measurements)
meas_var = 1e2
kf.R = np.array([[meas_var]])

num_samples = raw_accel.shape[0]

taxis = np.array(range(num_samples))
# Example loop for Kalman filter using accelerometer data
for t in range(num_samples):
    z = raw_accel[t][0]  # Use the X-axis acceleration in world frame
    z = 0 if (np.abs(z) < 0.1) else z 

    # Predict step (use accelerometer data)
    kf.predict()
    kf.update(z)

    # Get the estimated state (position, velocity)
    # kf.x[1] = kf.x[1]*0.99 if (np.abs(kf.x[1]) < 0.1) else kf.x[1]
    estimated_position, estimated_velocity, estimated_accel = kf.x
    p_estimates.append(estimated_position)
    v_estimates.append(estimated_velocity)
    a_estimates.append(estimated_accel)
    kalman_gains.append(kf.K)

    #print(f"Estimated Position: {estimated_position}, Estimated Velocity: {estimated_velocity}")



# Plot Position, Velocity, Acceleration in subplots (Kalman Filter estimates)
fig1, axs1 = plt.subplots(3, 1, figsize=(8, 10))
fig1.suptitle('Kalman Filter Estimates')

taxis = taxis * dt

# Position
p_estimates = np.array(p_estimates)
axs1[0].plot(taxis, p_estimates, label='KF position estimate')
axs1[0].set_ylabel('Position (m)')
axs1[0].grid()
axs1[0].legend()

# Velocity
v_estimates = np.array(v_estimates)
axs1[1].plot(taxis, v_estimates, label='KF velocity estimate')
axs1[1].set_ylabel('Velocity (m/s)')
axs1[1].grid()
axs1[1].legend()

# Accel
a_estimates = np.array(a_estimates)
axs1[2].plot(taxis, a_estimates, label='KF accel estimate')
axs1[2].set_ylabel('Acceleration (m/s^2)')
axs1[2].grid()
axs1[2].legend()

fig3 = plt.figure()
plt.plot(taxis, raw_accel, label=['raw accel x','raw accel y','raw accel z'])


plt.legend()


# Plot Kalman Gains in subplots
fig2, axs2 = plt.subplots(3, 1, figsize=(8, 10))
fig2.suptitle('Kalman Gains')

# Ensure the shape of kalman_gains is compatible
kalman_gains_array = np.array(kalman_gains)

# Gain for Position
axs2[0].plot(taxis, kalman_gains_array[:, 0, :], label='Kalman Gain Position')
axs2[0].set_ylabel('Position Gain')
axs2[0].grid()

# Gain for Velocity
axs2[1].plot(taxis, kalman_gains_array[:, 1, :], label='Kalman Gain Velocity')
axs2[1].set_ylabel('Velocity Gain')
axs2[1].grid()

# Gain for Accel
axs2[2].plot(taxis, kalman_gains_array[:, 2, :], label='Kalman Gain Accel')
axs2[2].set_ylabel('Accel Gain')
axs2[2].grid()

fig4 = plt.figure()
# Define the parameters
x_start = 2.1
x_end = 8.1
slope = 1.8 / 6
x_start_b = 10.6
x_end_b = 16.6

y = np.zeros(len(taxis))
# Calculate y values based on the line equation y = slope * (x - x_start)
y[int(x_start/dt):int(x_end/dt)] = slope * (taxis[int(x_start/dt):int(x_end/dt)] - x_start)
y[int(x_end/dt):int(x_start_b/dt)] = y[int(x_end/dt)-1]
y[int(x_start_b/dt):int(x_end_b/dt)] = -slope * (taxis[int(x_start_b/dt):int(x_end_b/dt)] - x_start_b) + y[int(x_end/dt)-1]

# Plot the line
plt.plot(taxis, y, label=f'ideal constant velocity path')
plt.plot(taxis, p_estimates, label=f'kalman position estimate')
plt.plot(taxis, abs(y-p_estimates), label=f'error', c='red')


# Add labels and title
plt.xlabel('time (s)')
plt.ylabel('position (m)')
plt.title('Position estimate error as a function of time')
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.legend()

# Show the plot
plt.grid(True)
plt.show()


plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust layout and leave space for the title

plt.show()
