import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz

# Function to read and extract data from the file
def read_file(filename):
    time = []
    world_x = []
    world_y = []
    world_z = []

    with open(filename, 'r') as file:
        lines = file.readlines()
        for i in range(0, len(lines), 6):
            # Extracting time
            time_line = lines[i].strip()
            time_value = float(re.search(r"time: ([\d\.]+)", time_line).group(1))
            time.append(time_value)

            # Extracting world values (x, y, z) and converting to meters per second squared
            world_line = lines[i+5].strip()
            world_values = re.search(r"world: \((-?[\d\.]+), (-?[\d\.]+), (-?[\d\.]+)\)", world_line)
            world_x.append(float(world_values.group(1)))
            world_y.append(float(world_values.group(2)))
            world_z.append(float(world_values.group(3)))

    return np.array(time), np.array(world_x), np.array(world_y), np.array(world_z)

# Function to calculate velocity and position for each axis
def calculate_velocity_position(time, acceleration):
    velocity = cumtrapz(acceleration, time, initial=0)  # Velocity is the integral of acceleration
    position = cumtrapz(velocity, time, initial=0)  # Position is the integral of velocity
    return velocity, position

# Function to plot world values, velocity, and position
def plot_data(time, world_x, world_y, world_z, velocity_x, position_x, position_y, position_z):
    fig = plt.figure(figsize=(12, 8))

    # Plotting the world values in 3D (acceleration)
    ax = fig.add_subplot(231, projection='3d')
    ax.plot(world_x, world_y, world_z, label='Movement in 3D space')
    ax.set_xlabel('World X (m/s²)')
    ax.set_ylabel('World Y (m/s²)')
    ax.set_zlabel('World Z (m/s²)')
    ax.legend()
    ax.set_title('3D World Movement (Acceleration)')

    # Plotting world x, velocity, and position
    ax1 = fig.add_subplot(232)
    ax1.plot(time, world_x, label='World X (acceleration)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.set_title('Acceleration in X-Axis')
    ax1.legend()

    ax2 = fig.add_subplot(233)
    ax2.plot(time, velocity_x, label='Velocity X', color='g')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity in X-Axis')
    ax2.legend()

    ax3 = fig.add_subplot(234)
    ax3.plot(time, position_x, label='Position X', color='r')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position (m)')
    ax3.set_title('Position in X-Axis')
    ax3.legend()

    # Plotting 3D position
    ax4 = fig.add_subplot(235, projection='3d')
    ax4.plot(position_x, position_y, position_z, label='3D Position')
    ax4.set_xlabel('Position X (m)')
    ax4.set_ylabel('Position Y (m)')
    ax4.set_zlabel('Position Z (m)')
    ax4.legend()
    ax4.set_title('3D Position Movement')

    plt.tight_layout()
    plt.show()

# Main function
def main():
    # Read the data
    time, world_x, world_y, world_z = read_file('lab1/fixed_rot.txt')  # Replace 'data.txt' with your file

    # Calculate velocity and position for each axis
    velocity_x, position_x = calculate_velocity_position(time, world_x)
    velocity_y, position_y = calculate_velocity_position(time, world_y)
    velocity_z, position_z = calculate_velocity_position(time, world_z)

    # Plot the data
    plot_data(time, world_x, world_y, world_z, velocity_x, position_x, position_y, position_z)

if __name__ == '__main__':
    main()
