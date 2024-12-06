import numpy as np

# Read the file and extract the coordinates
with open("/home/patricia/centroid_20240705_003514.txt", "r") as file:
    coordinates = [eval(line.strip()) for line in file]

# Separate x and y coordinates
x_coords = [coord[0] for coord in coordinates]
y_coords = [coord[1] for coord in coordinates]

# Calculate mean centroid position
mean_x = np.mean(x_coords)
mean_y = np.mean(y_coords)

# Calculate Euclidean distance of each point from the mean centroid
distances = [np.sqrt((x - mean_x)**2 + (y - mean_y)**2) for x, y in coordinates]

# Calculate the variance of these distances
combined_variance = np.var(distances)

print(f"Variance in x-coordinates: {np.var(x_coords)}")
print(f"Variance in y-coordinates: {np.var(y_coords)}")
print(f"Combined variance in centroid position: {combined_variance}")