import ICM20948 as ICM
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import find_peaks

# Function to calculate roll, pitch, and yaw from accelerometer
def calculate_roll_pitch_yaw_accel(accel):
    ax, ay, az = accel
    roll = np.arctan2(ay, az) * 180 / np.pi  # Roll angle in degrees
    pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2)) * 180 / np.pi  # Pitch angle in degrees
    yaw = np.arctan2(ay, ax) * 180 / np.pi  # Approximate Yaw angle in degrees (limited accuracy without magnetometer)
    return roll, pitch, yaw

# Function to integrate gyro data to estimate roll, pitch, and yaw
def integrate_gyro(gyro, dt, current_angles):
    roll, pitch, yaw = current_angles
    roll += gyro[0] * dt  # Integrate angular velocity for roll
    pitch += gyro[1] * dt  # Integrate angular velocity for pitch
    yaw += gyro[2] * dt    # Integrate angular velocity for yaw
    return roll, pitch, yaw

def init():
    ax_roll.set_xlim(0, 100)  
    ax_roll.set_ylim(-180, 180)
    ax_pitch.set_xlim(0, 100)
    ax_pitch.set_ylim(-180, 180)
    ax_yaw.set_xlim(0, 100)
    ax_yaw.set_ylim(-180, 180)
    
    ax_roll.legend()
    ax_pitch.legend()
    ax_yaw.legend()
    
    return ln_roll_accel, ln_roll_gyro, ln_pitch_accel, ln_pitch_gyro, ln_yaw_accel, ln_yaw_gyro

# Function to detect peaks and valleys (minima)
def detect_peaks_and_valleys(y_data):
    peaks, _ = find_peaks(y_data)  # Detect peaks (maxima)
    valleys, _ = find_peaks([-y for y in y_data])  # Detect valleys (minima)
    return peaks, valleys

# Function to annotate peaks on the plot
def annotate_peaks(ax, x_data, y_data, peaks, valleys, label):
    for peak in peaks:
        ax.annotate(f'{label} Peak: {y_data[peak]:.2f}', xy=(x_data[peak], y_data[peak]),
                    xytext=(x_data[peak], y_data[peak] + 10),
                    arrowprops=dict(facecolor='green', shrink=0.05))
    for valley in valleys:
        ax.annotate(f'{label} Valley: {y_data[valley]:.2f}', xy=(x_data[valley], y_data[valley]),
                    xytext=(x_data[valley], y_data[valley] - 10),
                    arrowprops=dict(facecolor='blue', shrink=0.05))

# Update function for the animation
def update(frame):
    global prev_time
    current_time = time.time()
    dt = current_time - prev_time  # Calculate the time step (dt) since the last frame
    prev_time = current_time
    
    icm20948.icm20948_Gyro_Accel_Read()
    
    accel_adc = [ICM.Accel[0], ICM.Accel[1], ICM.Accel[2]]
    gyro_adc = [ICM.Gyro[0], ICM.Gyro[1], ICM.Gyro[2]]
    
    # Convert sensor data to real-world units
    accel = [accel_adc[i] * (2.0 / 2**15) for i in range(3)]  # Convert accel to g
    gyro = [gyro_adc[i] * (1000.0 / 2**15) for i in range(3)]  # Convert gyro to dps (degrees/second)

    # Calculate roll, pitch, and yaw from accelerometer
    roll_accel, pitch_accel, yaw_accel = calculate_roll_pitch_yaw_accel(accel)
    
    # Integrate gyro data for roll, pitch, and yaw
    global current_angles
    roll_gyro, pitch_gyro, yaw_gyro = integrate_gyro(gyro, dt, current_angles)
    current_angles = (roll_gyro, pitch_gyro, yaw_gyro)  # Update current angles for next integration step
    
    # Update plot data
    x_data.append(frame)
    y_data_roll_accel.append(roll_accel)
    y_data_pitch_accel.append(pitch_accel)
    y_data_yaw_accel.append(yaw_accel)
    
    y_data_roll_gyro.append(roll_gyro)
    y_data_pitch_gyro.append(pitch_gyro)
    y_data_yaw_gyro.append(yaw_gyro)
    
    # Keep data in range of 100 points
    if len(x_data) > 100:
        x_data.pop(0)
        y_data_roll_accel.pop(0)
        y_data_pitch_accel.pop(0)
        y_data_yaw_accel.pop(0)
        y_data_roll_gyro.pop(0)
        y_data_pitch_gyro.pop(0)
        y_data_yaw_gyro.pop(0)

    # Detect peaks and valleys for each line
    peaks_roll_accel, valleys_roll_accel = detect_peaks_and_valleys(y_data_roll_accel)
    peaks_pitch_accel, valleys_pitch_accel = detect_peaks_and_valleys(y_data_pitch_accel)
    peaks_yaw_accel, valleys_yaw_accel = detect_peaks_and_valleys(y_data_yaw_accel)
    
    peaks_roll_gyro, valleys_roll_gyro = detect_peaks_and_valleys(y_data_roll_gyro)
    peaks_pitch_gyro, valleys_pitch_gyro = detect_peaks_and_valleys(y_data_pitch_gyro)
    peaks_yaw_gyro, valleys_yaw_gyro = detect_peaks_and_valleys(y_data_yaw_gyro)

    # Clear axes and plot new data
    ax_roll.clear()
    ax_pitch.clear()
    ax_yaw.clear()
    
    ax_roll.plot(x_data, y_data_roll_accel, 'r-', label="Roll (Accel)")
    ax_roll.plot(x_data, y_data_roll_gyro, 'r--', label="Roll (Gyro)")
    annotate_peaks(ax_roll, x_data, y_data_roll_accel, peaks_roll_accel, valleys_roll_accel, "Accel")
    annotate_peaks(ax_roll, x_data, y_data_roll_gyro, peaks_roll_gyro, valleys_roll_gyro, "Gyro")
    
    ax_pitch.plot(x_data, y_data_pitch_accel, 'b-', label="Pitch (Accel)")
    ax_pitch.plot(x_data, y_data_pitch_gyro, 'b--', label="Pitch (Gyro)")
    annotate_peaks(ax_pitch, x_data, y_data_pitch_accel, peaks_pitch_accel, valleys_pitch_accel, "Accel")
    annotate_peaks(ax_pitch, x_data, y_data_pitch_gyro, peaks_pitch_gyro, valleys_pitch_gyro, "Gyro")
    
    ax_yaw.plot(x_data, y_data_yaw_accel, 'g-', label="Yaw (Accel)")
    ax_yaw.plot(x_data, y_data_yaw_gyro, 'g--', label="Yaw (Gyro)")
    annotate_peaks(ax_yaw, x_data, y_data_yaw_accel, peaks_yaw_accel, valleys_yaw_accel, "Accel")
    annotate_peaks(ax_yaw, x_data, y_data_yaw_gyro, peaks_yaw_gyro, valleys_yaw_gyro, "Gyro")
    
    return ln_roll_accel, ln_roll_gyro, ln_pitch_accel, ln_pitch_gyro, ln_yaw_accel, ln_yaw_gyro

if __name__ == '__main__':
    print("\nSense HAT Test Program ...\n")
    icm20948 = ICM.ICM20948()
    
    # Initialize current angles for gyro integration
    current_angles = (0.0, 0.0, 0.0)  # roll, pitch, yaw (starts at 0 degrees)

    # Initialize time tracking for dt
    prev_time = time.time()

    # Set up the plot with 3 subplots
    fig, (ax_roll, ax_pitch, ax_yaw) = plt.subplots(3, 1, figsize=(8, 10))

    x_data = []
    y_data_roll_accel, y_data_pitch_accel, y_data_yaw_accel = [], [], []
    y_data_roll_gyro, y_data_pitch_gyro, y_data_yaw_gyro = [], [], []

    ln_roll_accel, = ax_roll.plot([], [], 'r-', label="Roll (Accel)")
    ln_roll_gyro, = ax_roll.plot([], [], 'r--', label="Roll (Gyro)")
   


    # Set up animation
    ani = FuncAnimation(fig, update, frames=np.linspace(0, 100, 100), init_func=init, blit=True, interval=100)

    plt.tight_layout()

    # Save the animation as a GIF or as a static image
    ani.save('/home/patricia/lab1/roll_pitch_yaw_animation.gif', writer='imagemagick', fps=10)
    plt.savefig('/home/patricia/lab1/roll_pitch_yaw_plot.png')
    
    print("Plot saved as '/home/patricia/lab1/roll_pitch_yaw_plot.png' or GIF saved as '/home/patricia/lab1/roll_pitch_yaw_animation.gif'.")

