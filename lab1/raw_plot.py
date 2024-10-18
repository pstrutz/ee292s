import matplotlib.pyplot as plt
import re
import time

# Function to read and parse the data from the file
def read_sensor_data(file_path):
    accel_data = {'X': [], 'Y': [], 'Z': []}
    gyro_data = {'X': [], 'Y': [], 'Z': []}
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    for line in lines:
        if 'Accel' in line:
            # Extract X, Y, Z values for Accel using regular expressions
            match = re.search(r'Accel:\s*X\s*=\s*([-.\d]+)\s*,\s*Y\s*=\s*([-.\d]+)\s*,\s*Z\s*=\s*([-.\d]+)', line)
            if match:
                accel_data['X'].append(float(match.group(1)))
                accel_data['Y'].append(float(match.group(2)))
                accel_data['Z'].append(float(match.group(3)))
        elif 'Gyro' in line:
            # Extract X, Y, Z values for Gyro using regular expressions
            match = re.search(r'Gyro:\s*X\s*=\s*([-.\d]+)\s*,\s*Y\s*=\s*([-.\d]+)\s*,\s*Z\s*=\s*([-.\d]+)', line)
            if match:
                gyro_data['X'].append(float(match.group(1)))
                gyro_data['Y'].append(float(match.group(2)))
                gyro_data['Z'].append(float(match.group(3)))
    
    return accel_data, gyro_data

# Function to plot the data in real-time
def plot_data(accel_data, gyro_data, interval=0.1):
    plt.ion()  # Interactive mode ON
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1.set_title('Accelerometer Data')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Acceleration (g)')
    
    ax2.set_title('Gyroscope Data')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (°/s)')
    
    accel_x_line, = ax1.plot([], [], label='X')
    accel_y_line, = ax1.plot([], [], label='Y')
    accel_z_line, = ax1.plot([], [], label='Z')
    
    gyro_x_line, = ax2.plot([], [], label='X')
    gyro_y_line, = ax2.plot([], [], label='Y')
    gyro_z_line, = ax2.plot([], [], label='Z')
    
    ax1.legend()
    ax2.legend()
    
    accel_x_data, accel_y_data, accel_z_data = [], [], []
    gyro_x_data, gyro_y_data, gyro_z_data = [], [], []
    time_data = []
    
    for i in range(len(accel_data['X'])):
        time_data.append(i * interval)
        
        # Update accelerometer data
        accel_x_data.append(accel_data['X'][i])
        accel_y_data.append(accel_data['Y'][i])
        accel_z_data.append(accel_data['Z'][i])
        
        # Update gyroscope data
        gyro_x_data.append(gyro_data['X'][i])
        gyro_y_data.append(gyro_data['Y'][i])
        gyro_z_data.append(gyro_data['Z'][i])
        
        # Set data for the plot lines
        accel_x_line.set_data(time_data, accel_x_data)
        accel_y_line.set_data(time_data, accel_y_data)
        accel_z_line.set_data(time_data, accel_z_data)
        
        gyro_x_line.set_data(time_data, gyro_x_data)
        gyro_y_line.set_data(time_data, gyro_y_data)
        gyro_z_line.set_data(time_data, gyro_z_data)
        
        # Rescale the axes
        ax1.relim()
        ax1.autoscale_view()
        
        ax2.relim()
        ax2.autoscale_view()
        
        plt.draw()
        plt.pause(interval)  # Pause for the specified interval (0.1s for 10Hz)
    
    plt.ioff()  # Turn off interactive mode
    plt.show()

# Main function to run the program
def main():
    file_path = '/Users/egeturan/Documents/Sensing/SmartphoneSensors292S/ee292s/lab1/p2_raw_test4.txt'  # Replace with your file path
    accel_data, gyro_data = read_sensor_data(file_path)
    
    # Plot the data at 10Hz (0.1s between each data point)
    plot_data(accel_data, gyro_data, interval=0.1)

if __name__ == '__main__':
    main()
