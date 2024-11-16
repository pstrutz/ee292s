import time
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import ADS1256
import RPi.GPIO as GPIO

def generate_prbs(polynomial, length, seed):
    """Generate a Pseudo-Random Bit Sequence (PRBS) based on a given polynomial and length.
    
    Args:
        polynomial (int): Polynomial used for the feedback in the LFSR (e.g., 0b1100000 for PRBS7).
        length (int): The length of the PRBS sequence (e.g., 127 for PRBS7).
        seed (int): Initial seed (non-zero) for the LFSR.
        
    Returns:
        list: List containing the PRBS sequence as binary bits.
    """
    num_bits = polynomial.bit_length() 
    lfsr = [int(bit) for bit in format(seed, f'0{num_bits}b')]
    prbs_sequence = []

    for _ in range(length):
        
        # Output the last bit in the register (can be any bit depending on the design)
        prbs_sequence.append(lfsr[-1])
        # print(lfsr, prbs_sequence[-1])
        # XOR the tapped bits to calculate feedback bit
        feedback = 0
        for bit_position in range(num_bits):
            # print("pos: ", bit_position)
            if (polynomial >> bit_position) & 1:  # Check if this position has a tap
                feedback ^= lfsr[-(bit_position + 1)]
                
        # Shift the register to the right and insert the feedback bit at the front
        lfsr = lfsr[1:] + [feedback]
        
    return prbs_sequence

def circular_cross_correlation(x, y):
    """Compute the circular cross-correlation of two 1D signals of arbitrary lengths.
    
    Args:
        x (np.ndarray): First input signal array.
        y (np.ndarray): Second input signal array.
        
    Returns:
        np.ndarray: Circular cross-correlation result.
    """
    # Determine the length of the result, which will match the length of the longer input
    N = max(len(x), len(y))
    
    # Zero-pad both sequences to the same length
    x = np.pad(x, (0, N - len(x)), mode='constant')
    y = np.pad(y, (0, N - len(y)), mode='constant')
    
    result = np.zeros(N)
    
    # Compute circular cross-correlation
    for k in range(N):
        result[k] = np.sum(x * np.roll(y, k))
    
    return result

# Drive and sense line pins as wired
DRIVE = [21, 7, 12, 16, 20]
SENSE = [1, 2, 3, 4, 5, 6, 7]

import time
import ADS1256
import RPi.GPIO as GPIO    

if __name__ == '__main__':
    ADC = ADS1256.ADS1256()
    ADC.ADS1256_init()

    # initialization
    ADDR = 0x00
    ON = 0x02
    ADC.ADS1256_WriteReg(ADDR, ON)

    # GPIO setup
    # GPIO.setmode(GPIO.BOARD)
    GPIO.setup(DRIVE, GPIO.OUT)

    polynomial = 0xb8
    # length = 2**16-1
    length = 2**8-1
    shift = length // 5
    seed = 0x01
    prbs0 = generate_prbs(polynomial, length, seed)
    prbs1 = np.roll(prbs0, 1*shift)
    prbs2 = np.roll(prbs0, 2*shift)
    prbs3 = np.roll(prbs0, 3*shift)
    prbs4 = np.roll(prbs0, 4*shift)
    prbs = np.vstack([prbs0, prbs1, prbs2, prbs3, prbs4])

    raw_sense = np.zeros((len(SENSE), length))

    # Record the start time
    start_time = time.time()
    # Target frequency and interval
    target_freq = 15 * 10**3  # Hz
    target_period = 1.0 / target_freq  # Interval in seconds


    xcor_raw = np.zeros((np.shape(raw_sense)[0], len(prbs0)))
    
    print("Setting up the plot")

    # Create a figure and axis for live updating all 7 rows of xcor_raw on the same graph
    figure, ax = plt.subplots(figsize=(10, 6))

    # Create a line plot for each row in xcor_raw
    lines = []
    for idx in range(7):
        line, = ax.plot(xcor_raw[idx], label=f"Sense Line {idx + 1}")
        lines.append(line)

    ax.set_ylim(150, 300)  # Adjust y-axis limits as needed
    ax.set_title("Live Plot of Cross Correlations of Sense Lines")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Amplitude")
    ax.legend(loc='upper right')
    ax.grid(True)

    # Draw and cache the background
    figure.canvas.draw()
    axbackground = figure.canvas.copy_from_bbox(ax.bbox)
    plt.show(block=False)
    print("Plot setup successful")


    s = 0



    while True:
        for j in range(len(SENSE)):
            # Set GPIO outputs based on PRBS
            
            for i in range(len(DRIVE)):
                GPIO.output(DRIVE[i], int(prbs[i, s]))


            # One by one
            ADC.ADS1256_SetChannal(SENSE[j])
            ADC_Value = ADC.ADS1256_GetChannalValue(SENSE[j])
                
            loop_start_time = time.time()  # Record the time when the loop starts
                
        
            raw_sense[j, s] = ADC_Value * 5.0 / 0x7fffff
                
            s = s + 1
                
            if (s >= length):

                for idx in range(np.shape(raw_sense)[0]):
                    xcor_raw[idx] = circular_cross_correlation(prbs0, raw_sense[idx, :])
                
                s = 0
                
                 # Update the line plots with the new data for each row
                for idx, line in enumerate(lines):
                    line.set_ydata(xcor_raw[idx])

                # Restore the background and redraw the updated lines
                figure.canvas.restore_region(axbackground)
                for line in lines:
                    ax.draw_artist(line)

                figure.canvas.blit(ax.bbox)
                figure.canvas.flush_events()

            else: 
                elapsed_time = time.time() - start_time

                # Calculate loop duration
                loop_duration = time.time() - loop_start_time

                # Sleep for the remaining time to maintain 10Hz rate
                if loop_duration < target_period:
                    time.sleep(target_period - loop_duration)








