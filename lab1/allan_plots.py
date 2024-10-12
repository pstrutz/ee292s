import numpy as np
import matplotlib.pyplot as plt
import re

def parse_sensor_data(file_path):
    """Parses the sensor data from the given file."""
    time = []
    accel_data = {'roll': [], 'pitch': [], 'yaw': [], 'tilt': []}
    gyro_data = {'roll': [], 'pitch': [], 'yaw': [], 'tilt': []}
    filtered_tilt = []

    with open(file_path, 'r') as file:
        content = file.read()

    # Regular expressions to match the lines
    time_pattern = r'time: ([\d\.]+)'
    accel_pattern = r'acce: \(([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+)\)'
    gyro_pattern = r'gyro: \(([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+)\)'
    accel_tilt_pattern = r'acce tilt: ([\d\.\-]+)'
    gyro_tilt_pattern = r'gyro tilt: ([\d\.\-]+)'
    filtered_tilt_pattern = r'filtered tilt: ([\d\.\-]+)'

    # Find all matches
    times = re.findall(time_pattern, content)
    accels = re.findall(accel_pattern, content)
    gyros = re.findall(gyro_pattern, content)
    accel_tilts = re.findall(accel_tilt_pattern, content)
    gyro_tilts = re.findall(gyro_tilt_pattern, content)
    filtered_tilts = re.findall(filtered_tilt_pattern, content)

    # Parse and store data
    time = [float(t) for t in times]
    for i in range(len(accels)):
        accel_data['roll'].append(float(accels[i][0]))
        accel_data['pitch'].append(float(accels[i][1]))
        accel_data['yaw'].append(float(accels[i][2]))
        gyro_data['roll'].append(float(gyros[i][0]))
        gyro_data['pitch'].append(float(gyros[i][1]))
        gyro_data['yaw'].append(float(gyros[i][2]))
        accel_data['tilt'].append(float(accel_tilts[i]))
        gyro_data['tilt'].append(float(gyro_tilts[i]))
        filtered_tilt.append(float(filtered_tilts[i]))

    return time, accel_data, gyro_data, filtered_tilt

def allan_deviation(data, max_tau=None):
    """Computes Allan deviation for the given data."""
    N = len(data)
    if max_tau is None:
        max_tau = N // 2  # Set a maximum tau (half the data length by default)

    tau_values = np.logspace(0, np.log10(max_tau), num=100, dtype=int)
    tau_values = np.unique(tau_values)  # Remove duplicates

    allan_devs = []

    for tau in tau_values:
        if tau >= len(data):  # Skip tau if it's larger than the data size
            break
        # Calculate the cluster averages for each interval of length tau
        cluster_avgs = np.array([np.mean(data[i:i+tau]) for i in range(0, N - tau, tau)])
        
        # Compute the Allan variance
        allan_var = 0.5 * np.mean(np.diff(cluster_avgs) ** 2)
        
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
    

def main():
    # Load data from the file
    file_path = '/Users/patriciastrutz/Library/CloudStorage/OneDrive-Stanford/Stanford/24-25a Fall/EE292S/Lab1/test.txt'  # Path to your sensor data file
    time, accel_data, gyro_data, filtered_tilt = parse_sensor_data(file_path)

    # Process accelerometer data (roll, pitch, yaw)
    accel_labels = ['Accel Roll', 'Accel Pitch', 'Accel Yaw']
    accel_tau_values = []
    accel_allan_devs = []
    for axis in ['roll', 'pitch', 'yaw']:
        tau, allan_dev = allan_deviation(accel_data[axis])
        accel_tau_values.append(tau)
        accel_allan_devs.append(allan_dev)

    # Process gyroscope data (roll, pitch, yaw)
    gyro_labels = ['Gyro Roll', 'Gyro Pitch', 'Gyro Yaw']
    gyro_tau_values = []
    gyro_allan_devs = []
    for axis in ['roll', 'pitch', 'yaw']:
        tau, allan_dev = allan_deviation(gyro_data[axis])
        gyro_tau_values.append(tau)
        gyro_allan_devs.append(allan_dev)

    # Process tilt data (acce tilt, gyro tilt, filtered tilt)
    tilt_labels = ['Accel Tilt', 'Gyro Tilt', 'Filtered Tilt']
    tilt_tau_values = []
    tilt_allan_devs = []
    for tilt_data in [accel_data['tilt'], gyro_data['tilt'], filtered_tilt]:
        tau, allan_dev = allan_deviation(tilt_data)
        tilt_tau_values.append(tau)
        tilt_allan_devs.append(allan_dev)

    # Plot accelerometer Allan deviations
    plot_allan_deviation(accel_tau_values, accel_allan_devs, accel_labels, 'Allan Deviation - Accelerometer')

    # Plot gyroscope Allan deviations
    plot_allan_deviation(gyro_tau_values, gyro_allan_devs, gyro_labels, 'Allan Deviation - Gyroscope')

    # Plot tilt Allan deviations
    plot_allan_deviation(tilt_tau_values, tilt_allan_devs, tilt_labels, 'Allan Deviation - Tilt')

    plt.show()

if __name__ == "__main__":
    main()
