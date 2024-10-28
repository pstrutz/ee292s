#!/Users/s/bin/python3
# Ege, 11/11/2024

import sys
import math as m
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from matplotlib.collections import LineCollection
import matplotlib.colors as mcolors

# Read sensor values from text file
def extract_values_from_file(file_path):
    """
    This function reads a text file and extracts accelerometer and gyroscope values.
    Args:
    - file_path: Path to the text file.
    Returns:
    - a_values: List of accelerometer readings.
    - g_values: List of gyroscope readings.
    """
    a_values = []
    g_values = []
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    for line in lines:
        if line.startswith("a:"):
            a_value = [float(x) for x in line.split(": (")[1].replace(")", "").split(",")]
            a_values.append(a_value)
        elif line.startswith("g:"):
            g_value = [float(x) for x in line.split(": (")[1].replace(")", "").split(",")]
            g_values.append(g_value)
    
    return a_values, g_values

# Constants and initial parameters
dt = 0.01  # time step
GEE_TO_METERS = 9.81  # 1 gee = 9.81 m/s²
meas_var = GEE_TO_METERS**2 * (0.01)**2  # Measurement variance

# Load sensor data
raw_measurements_accel, raw_measurements_gyro = extract_values_from_file("/Users/egeturan/Documents/Sensing/SmartphoneSensors292S/ee292s/lab1/oct23/2D_L")
raw_accel = 9.81 * np.array(raw_measurements_accel)  # Convert to m/s^2
raw_gyro = np.array(raw_measurements_gyro)  # Angular velocities

# Kalman Filter initialization for 2D movement with yaw
kf = KalmanFilter(dim_x=7, dim_z=3)  # State: x, vx, ax, y, vy, ay, yaw

# State transition matrix
kf.F = np.array([
    [1, dt, 0.5*dt**2, 0, 0, 0, 0],
    [0, 1, dt, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, dt, 0.5*dt**2, 0],
    [0, 0, 0, 0, 1, dt, 0],
    [0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1]  # Yaw evolves independently as per gyro z-axis
])

# Measurement matrix (we measure x, y acceleration, and yaw rate)
kf.H = np.array([
    [0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 1]
])

# Initial state and covariance
kf.x = np.zeros(7)
kf.P = np.eye(7) * 1e-2
kf.Q = np.eye(7) * 1e-2
kf.R = np.array([[meas_var, 0, 0], [0, meas_var, 0], [0, 0, 1e-2]])

# Arrays for storing results
p_estimates_x, p_estimates_y = [], []
yaw_estimates = []
num_samples = raw_accel.shape[0]

# Kalman Filter loop
for t in range(num_samples):
    accel_x, accel_y = raw_accel[t][0], raw_accel[t][1]
    yaw_rate_z = raw_gyro[t][2]
    
    # Predict step
    kf.predict()
    
    # Update step with accelerations affected by yaw
    yaw = kf.x[6]
    
    # Rotate accelerations to the global frame based on the current yaw
    global_accel_x = accel_x * np.cos(yaw) - accel_y * np.sin(yaw)
    global_accel_y = accel_x * np.sin(yaw) + accel_y * np.cos(yaw)

    # Measurement update with global accelerations and yaw rate
    z = np.array([global_accel_x, global_accel_y, yaw_rate_z])
    kf.update(z)
    
    # Store estimated states
    estimated_position_x, estimated_velocity_x, _, estimated_position_y, estimated_velocity_y, _, estimated_yaw = kf.x
    p_estimates_x.append(estimated_position_x)
    p_estimates_y.append(estimated_position_y)
    yaw_estimates.append(estimated_yaw)

# Plot 2D trajectory with color-coded time progression
taxis = np.array(range(num_samples)) * dt  # Time axis

# Create segments for color-coding
points = np.array([p_estimates_x, p_estimates_y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = mcolors.Normalize(vmin=0, vmax=len(taxis))
lc = LineCollection(segments, cmap='viridis', norm=norm)
lc.set_array(np.arange(len(taxis)))
lc.set_linewidth(2)

# Plot 2D trajectory
fig, ax = plt.subplots()
ax.add_collection(lc)
plt.colorbar(lc, ax=ax, label='Time Progression')

# Plot start and end points
initial_position = np.array([p_estimates_x[0], p_estimates_y[0]])
final_position = np.array([p_estimates_x[-1], p_estimates_y[-1]])
error_distance = np.linalg.norm(final_position - initial_position)

ax.scatter(initial_position[0], initial_position[1], color='green', label='Start Position')
ax.scatter(final_position[0], final_position[1], color='red', label='End Position')

# Annotate error with arrow from start to end position
plt.annotate(f'Error: {error_distance:.2f} m', 
             xy=final_position, 
             xytext=initial_position,
             arrowprops=dict(facecolor='red', arrowstyle='->'))

# Labels and grid
ax.set_xlabel('Position X (m)')
ax.set_ylabel('Position Y (m)')
ax.set_title('2D Trajectory with Yaw Compensation in Kalman Filter')
ax.grid(True)
ax.legend()
plt.show()
