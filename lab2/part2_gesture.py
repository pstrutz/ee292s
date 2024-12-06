import time
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from filterpy.kalman import KalmanFilter

# Helper functions
def generate_prbs(polynomial, length, seed):
    num_bits = polynomial.bit_length() 
    lfsr = [int(bit) for bit in format(seed, f'0{num_bits}b')]
    prbs_sequence = []
    for _ in range(length):
        prbs_sequence.append(lfsr[-1])
        feedback = 0
        for bit_position in range(num_bits):
            if (polynomial >> bit_position) & 1:
                feedback ^= lfsr[-(bit_position + 1)]
        lfsr = lfsr[1:] + [feedback]
    return prbs_sequence

def circular_cross_correlation(x, y):
    N = max(len(x), len(y))
    x = np.pad(x, (0, N - len(x)), mode='constant')
    y = np.pad(y, (0, N - len(y)), mode='constant')
    result = np.zeros(N)
    for k in range(N):
        result[k] = np.sum(x * np.roll(y, k))
    return result

def fit_gaussian_ellipse(touch_map, num_points=3):
    flat_map = touch_map.flatten()
    sorted_indices = np.argsort(flat_map)[-num_points:]
    max_indices = np.unravel_index(sorted_indices, touch_map.shape)
    weights = flat_map[sorted_indices]
    centroid_x = np.sum(max_indices[1] * weights) / np.sum(weights)
    centroid_y = np.sum(max_indices[0] * weights) / np.sum(weights)
    centroid = (centroid_x, centroid_y)
    x = np.arange(touch_map.shape[1])
    y = np.arange(touch_map.shape[0])
    X, Y = np.meshgrid(x, y)
    var_x = np.sum(touch_map * (X - centroid_x) ** 2) / np.sum(touch_map)
    var_y = np.sum(touch_map * (Y - centroid_y) ** 2) / np.sum(touch_map)
    major_axis_length = 2 * np.sqrt(var_x)
    minor_axis_length = 2 * np.sqrt(var_y)
    return centroid, major_axis_length, minor_axis_length

def initialize_kalman_filter():
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.x = np.array([0, 0, 0, 0])  # Initial state [x, vx, y, vy]
    kf.F = np.array([[1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]])
    kf.H = np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0]])
    kf.P *= 1000
    kf.R *= 5
    kf.Q *= 0.01
    return kf

# Drive and sense line pins as wired
DRIVE = [21, 7, 12, 16, 20]
SENSE = [1, 2, 3, 4, 5, 6, 7]
SETUP_SPI = True

# PRBS generation
polynomial = 0xb8
length = 2**8 - 1
shift = length // 5
seed = 0x01
prbs0 = generate_prbs(polynomial, length, seed)
prbs = np.vstack([np.roll(prbs0, i * shift) for i in range(len(DRIVE))])

# Data storage
raw_sense = np.zeros((7, length))
xcor = np.zeros((np.shape(raw_sense)[0], len(DRIVE)))
xcor_raw = np.zeros((np.shape(raw_sense)[0], len(prbs0)))
threshold = 210

# Kalman filter initialization
kf = initialize_kalman_filter()

# Figures
figure, ax = plt.subplots()
heatmap = ax.imshow(xcor, cmap='viridis', interpolation='nearest', vmin=threshold, vmax=length + 20)
plt.colorbar(heatmap)
plt.title("Live Updating Heatmap")
figure.canvas.draw()
axbackground = figure.canvas.copy_from_bbox(ax.bbox)
ax.invert_yaxis()
ax.invert_xaxis()
plt.show(block=False)

ellipse_fig, ellipse_ax = plt.subplots()
ellipse_ax.set_aspect('equal')
ellipse_ax.set_xlim(0, len(DRIVE))
ellipse_ax.set_ylim(0, len(SENSE))
ellipse_ax.set_title("Ellipse Fit Plot")
ellipse_ax.invert_xaxis()
plt.show(block=False)

velocity_fig, velocity_ax = plt.subplots()
velocity_ax.set_xlim(0, len(DRIVE))
velocity_ax.set_ylim(0, len(SENSE))
velocity_ax.set_aspect('equal')
velocity_ax.set_title("Centroid and Velocity")
plt.show(block=False)

# Main loop
s = 0
prev_time = time.time()
while True:
    # Simulate sensor acquisition (replace with actual sensor code)
    for j in range(len(SENSE)):
        for i in range(len(DRIVE)):
            raw_sense[j][s] = np.random.random() * 5  # Simulated ADC value
        s += 1

        if s >= length:
            for idx in range(np.shape(raw_sense)[0]):
                xcor_raw[idx] = circular_cross_correlation(prbs0, raw_sense[idx])
                xcor[idx] = [xcor_raw[idx][i * shift] for i in range(len(DRIVE))]

            touch_map = xcor
            touch_map[touch_map < threshold] = 0
            heatmap.set_data(touch_map)
            centroid, major_axis, minor_axis = fit_gaussian_ellipse(touch_map, num_points=5)

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            kf.F[0, 1] = dt
            kf.F[2, 3] = dt
            kf.predict()
            kf.update([centroid[0], centroid[1]])
            smoothed_centroid = (kf.x[0], kf.x[2])
            velocity = (kf.x[1], kf.x[3])

            ellipse_ax.patches.clear()
            ellipse_ax.lines.clear()
            ellipse_only = patches.Ellipse(xy=smoothed_centroid, width=major_axis, height=minor_axis,
                                           edgecolor='blue', facecolor='none', linestyle='--', linewidth=1.5)
            ellipse_ax.add_patch(ellipse_only)
            ellipse_ax.plot(smoothed_centroid[0], smoothed_centroid[1], 'ro')
            ellipse_fig.canvas.draw()

            velocity_ax.patches.clear()
            velocity_ax.lines.clear()
            velocity_ax.plot(smoothed_centroid[0], smoothed_centroid[1], 'ro')
            velocity_ax.arrow(smoothed_centroid[0], smoothed_centroid[1],
                              velocity[0], velocity[1],
                              head_width=0.2, head_length=0.2, fc='blue', ec='blue')
            velocity_fig.canvas.draw()
            velocity_fig.canvas.flush_events()

            figure.canvas.restore_region(axbackground)
            ax.draw_artist(heatmap)
            figure.canvas.blit(ax.bbox)
            figure.canvas.flush_events()

            s = 0
