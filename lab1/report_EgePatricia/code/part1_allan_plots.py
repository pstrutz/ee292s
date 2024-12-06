import numpy as np
import matplotlib.pyplot as plt
import re

# Example usage:
# file_path = 'sensor_data.txt'
# accel_data, gyro_data = parse_sensor_data(file_path)
def parse_sensor_data(file_path):
    """Parses the simplified sensor data from the given file."""
    accel_data = {'X': [], 'Y': [], 'Z': []}
    gyro_data = {'X': [], 'Y': [], 'Z': []}

    with open(file_path, 'r') as file:
        content = file.read()

    # Regular expressions to match the lines
    accel_pattern = r'Accel:\s+X\s=\s([-\d\.]+)\s,\sY\s=\s([-\d\.]+)\s,\sZ\s=\s([-\d\.]+)'
    gyro_pattern = r'Gyro:\s+X\s=\s([-\d\.]+)\s,\sY\s=\s([-\d\.]+)\s,\sZ\s=\s([-\d\.]+)'

    # Find all matches
    accels = re.findall(accel_pattern, content)
    gyros = re.findall(gyro_pattern, content)

    # Parse and store data
    for accel in accels:
        accel_data['X'].append(float(accel[0]))
        accel_data['Y'].append(float(accel[1]))
        accel_data['Z'].append(float(accel[2]))

    for gyro in gyros:
        gyro_data['X'].append(float(gyro[0]))
        gyro_data['Y'].append(float(gyro[1]))
        gyro_data['Z'].append(float(gyro[2]))

    return accel_data, gyro_data

def allan_deviation(data, max_tau=None):
    """Computes Allan deviation for the given data."""
    N = len(data)
    if max_tau is None:
        max_tau = N // 2  # Set a maximum tau (half the data length by default)

    tau_values = np.logspace(0, np.log10(max_tau), num=1000, dtype=int)
    tau_values = np.unique(tau_values)  # Remove duplicates

    allan_devs = []

    for tau in tau_values:
        if tau >= len(data):  # Skip tau if it's larger than the data size
            break
        
        # Calculate the number of clusters K for this tau
        K = (N // tau) - 1
        
        if K < 2:  # Not enough data for this tau, skip it
            continue

        # Calculate the cluster averages for each interval of length tau
        cluster_avgs = np.array([np.mean(data[i:i+tau]) for i in range(0, N - tau, tau)])
        
        # Compute the Allan variance using the formula from the image
        allan_var = (1 / (2 * (K - 1))) * np.sum((cluster_avgs[1:] - cluster_avgs[:-1]) ** 2)
        
        # Allan deviation is the square root of the variance
        allan_devs.append(np.sqrt(allan_var))

    return tau_values[:len(allan_devs)], allan_devs

def plot_allan_deviation(tau_values_list, allan_devs_list, labels, title):
    """Plots Allan deviation for multiple signals."""
    plt.figure(figsize=(10, 6))
    for tau_values, dev, label in zip(tau_values_list, allan_devs_list, labels):
        plt.loglog(tau_values, dev, label=label)
    plt.xlabel('Tau (s)')
    plt.ylabel('Allan Deviation')
    plt.title(title)
    plt.legend()
    plt.grid(True)

def plot_single_allan_deviation(tau_values, allan_devs, label, title):
    """Plots Allan deviation for a single signal."""
    plt.figure(figsize=(10, 6))
    plt.loglog(tau_values, allan_devs, label=label)
    plt.xlabel('Tau (s)')
    plt.ylabel('Allan Deviation')
    plt.title(title)
    plt.legend()
    plt.grid(True)

def main():
    # Load data from the file
    file_path = '/Users/patriciastrutz/Library/CloudStorage/OneDrive-Stanford/Stanford/24-25a Fall/EE292S/ee292s/lab1/raw_data_for_allan/30min_raw_data_stationary_24C_indoors_small_room.txt'  # Path to your sensor data file
    accel_data, gyro_data = parse_sensor_data(file_path)

    # Process accelerometer data (roll, pitch, yaw)
    accel_labels = ['Accel X', 'Accel Y', 'Accel Z']
    accel_tau_values = []
    accel_allan_devs = []
    for axis in ['X', 'Y', 'Z']:
        tau, allan_dev = allan_deviation(accel_data[axis])
        accel_tau_values.append(tau)
        accel_allan_devs.append(allan_dev)

    # Process gyroscope data (roll, pitch, yaw)
    gyro_labels = ['Gyro X', 'Gyro Y', 'Gyro Z']
    gyro_tau_values = []
    gyro_allan_devs = []
    for axis in ['X', 'Y', 'Z']:
        tau, allan_dev = allan_deviation(gyro_data[axis])
        gyro_tau_values.append(tau)
        gyro_allan_devs.append(allan_dev)


    # Plot accelerometer Allan deviations
    plot_allan_deviation(accel_tau_values, accel_allan_devs, accel_labels, 'Allan Deviation - Accelerometer')

    # Plot gyroscope Allan deviations
    plot_allan_deviation(gyro_tau_values, gyro_allan_devs, gyro_labels, 'Allan Deviation - Gyroscope')
    

    plt.show()

if __name__ == "__main__":
    main()
