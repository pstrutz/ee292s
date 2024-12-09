import numpy as np
import matplotlib.pyplot as plt

# File containing the recorded ECG data
data_file = '/home/patricia/ecg_data_fs400_hrv_patricia.csv'

# Load the data
time_data = []
ecg_data = []

with open(data_file, 'r') as f:
    next(f)  # Skip header
    for line in f:
        time_val, ecg_val = line.strip().split(',')
        time_data.append(float(time_val))
        ecg_data.append(float(ecg_val))

# Convert to NumPy arrays
ecg_data = np.array(ecg_data)

# Sampling frequency (Hz)
f_s = 400  # Ensure this matches the recording file's sampling rate
fft_length = len(ecg_data)  # Length for FFT to resolve heart rate frequency (adjust if necessary)

# Manual Hamming window function
def hamming_window(N):
    return 0.54 - 0.46 * np.cos(2 * np.pi * np.arange(N) / (N - 1))

# Plot ECG signal
plt.figure(figsize=(12, 8))

# Subplot 1: ECG Signal
plt.subplot(2, 1, 1)
plt.plot(time_data, ecg_data-np.mean(ecg_data), label="ECG Signal")
plt.title("ECG Signal")
plt.xlabel("Time (seconds)")
plt.ylabel("Voltage (V)")
plt.grid(True)
plt.legend()

print(fft_length)
print(len(ecg_data))

# Subplot 2: FFT Analysis
if len(ecg_data) >= fft_length:
    # Select the last segment of data for FFT
    fft_signal = ecg_data[-fft_length:]

    # Apply the Hamming window
    window = hamming_window(fft_length)
    fft_signal_windowed = fft_signal * window

    # Perform the FFT
    fft_output = np.fft.fft(fft_signal_windowed)

    # Get the corresponding frequencies
    frequencies = np.fft.fftfreq(fft_length, d=1/f_s)
    
    # Magnitude spectrum (only positive frequencies)
    magnitude_spectrum = np.abs(fft_output[:fft_length // 2])
    positive_frequencies = frequencies[:fft_length // 2]

    # Plot FFT result
    plt.subplot(2, 1, 2)
    plt.plot(positive_frequencies[3:20*f_s], magnitude_spectrum[3:20*f_s], label="FFT Magnitude")
    plt.xlim(0, 3)  # Focus on heart rate frequency range (0.5–3 Hz)
    plt.ylim(0, max(magnitude_spectrum[3:]) * 1.1)  # Adjust y-axis for visibility
    plt.title("Heart Rate Frequency Spectrum")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.grid(True)
    plt.legend()

# Show the plots
plt.tight_layout()
plt.show()
