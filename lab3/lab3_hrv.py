import numpy as np
from scipy.signal import find_peaks
import math


# File containing the recorded ECG data
data_file = '/Users/patriciastrutz/Library/CloudStorage/OneDrive-Stanford/Stanford/24-25a Fall/EE292S/ee292s/lab3/ecg_data_fs400_hrv_patricia.csv'

# Load the data
time_data = []
ecg_data = []

with open(data_file, 'r') as f:
    next(f)  # Skip header
    for line in f:
        time_val, ecg_val = line.strip().split(',')
        time_data.append(float(time_val))
        ecg_data.append(float(ecg_val))

fs = 125  
time_step = 1 / fs

# Detect peaks in the PPG signal
peaks, _ = find_peaks(ecg_data, prominence=0.02)  

# Calculate time differences between consecutive peaks 
peak_intervals = np.diff(peaks) * time_step

# Calculate average heart rate (in BPM)
avg_interval = np.mean(peak_intervals)  # Mean interval in seconds
bpm = 60 / avg_interval  # Convert to BPM

# Calculate HRV metrics
max_hrv = np.max(peak_intervals) - np.min(peak_intervals)  # Max variability
rms_hrv = math.sqrt(np.mean((peak_intervals - avg_interval)**2))  # RMS variability

# Output results
print(f"Heart Rate (BPM): {bpm:.2f}")
print(f"Max HRV (ms): {max_hrv*1000:.4f}")
print(f"RMS HRV (ms): {rms_hrv*1000:.4f}")
