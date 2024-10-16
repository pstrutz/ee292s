import matplotlib.pyplot as plt

# Initialize lists to hold time and filtered tilt values
times = []
tilts = []

# Read data from file
with open('test_part1_complex.txt', 'r') as file:  # Assuming the file is named 'data.txt'
    for line in file:
        if 'time' in line:
            # Extract the time value
            time_value = float(line.split(': ')[1])
            times.append(time_value)
        elif 'filtered tilt' in line:
            # Extract the filtered tilt value
            tilt_value = float(line.split(': ')[1])
            tilts.append(tilt_value)

# Plotting the data
plt.plot(times, tilts, marker='o')
plt.title('Filtered Tilt vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Filtered Tilt (degrees)')
plt.grid(True)
plt.show()
