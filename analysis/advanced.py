import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.fft import rfft, rfftfreq

# --- CONFIGURATION ---
CSV_FILE = 'data/0423_1649.csv'
FS = 50.0                           # Sampling rate (Hz)

# --- TUNING PARAMETERS ---
INIT_CUTOFF = 10.0                   # Hz (Pre-smoothing for Plot 1)
SECOND_CUTOFF = 10.0                 # Hz (Deeper smoothing for the chosen axis in Plot 3)
WINDOW_SEC = 20.0                   # Rolling window to evaluate dominant axis
MIN_STROKE_TIME = 0.9               # Minimum seconds between strokes (Debounce limit = ~66 SPM)


def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)


def get_dominant_axis_index(w_x, w_y, w_z, fs):
    """
    Evaluates X, Y, and Z windows using FFT. Returns 1 (X), 2 (Y), or 3 (Z).
    """
    axes = {1: w_x, 2: w_y, 3: w_z}
    best_idx = 1
    best_score = -1.0
    
    for idx, data in axes.items():
        detrended = data - np.mean(data)
        n = len(detrended)
        if n == 0: continue
        
        yf = np.abs(rfft(detrended))
        xf = rfftfreq(n, 1 / fs)

        # Look only at plausible rowing frequencies (15 to 60 SPM -> 0.25 to 1.0 Hz)
        valid_idx = np.where((xf >= 0.25) & (xf <= 1.0))[0]
        if len(valid_idx) == 0:
            continue

        dominant_peak_val = np.max(yf[valid_idx])
        total_energy = np.sum(yf)
        score = dominant_peak_val / (total_energy + 1e-6)

        if score > best_score:
            best_score = score
            best_idx = idx

    return best_idx


def analyze_rowing_data(filename):
    print("--- 1. Loading Data ---")
    ts, ax, ay, az = [], [], [], []
    
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0] != 'IMU': continue
                if len(row) < 8: continue
                try:
                    t = int(row[1])
                    x = float(row[2]) / 100.0
                    y = float(row[3]) / 100.0
                    z = float(row[4]) / 100.0
                    ts.append(t); ax.append(x); ay.append(y); az.append(z)
                except ValueError: 
                    continue
    except FileNotFoundError:
        print(f"File {filename} not found.")
        return

    time_sec = (np.array(ts) - ts[0]) / 1000.0
    accel_raw = np.column_stack((ax, ay, az))

    print("--- 2. Initial Smoothing ---")
    accel_smooth = np.zeros_like(accel_raw)
    for i in range(3):
        accel_smooth[:, i] = butter_lowpass_filter(accel_raw[:, i], INIT_CUTOFF, FS)


    print("--- 3. Tracking Dominant Axis & Stitching Data ---")
    window_samples = int(WINDOW_SEC * FS)
    step_samples = int(1.0 * FS) # Evaluate every 1 second
    
    # Initialize arrays for the continuous tracker and stitched data
    dom_axis_array = np.zeros(len(time_sec), dtype=int)
    chosen_raw_stitched = np.zeros(len(time_sec))

    # Get the starting axis (using the first 10 seconds)
    current_best_idx = get_dominant_axis_index(
        accel_smooth[0:window_samples, 0], 
        accel_smooth[0:window_samples, 1], 
        accel_smooth[0:window_samples, 2], FS)

    for i in range(len(time_sec)):
        # Update our decision every 1 second, looking backward 10 seconds
        if i >= window_samples and i % step_samples == 0:
            ax_w = accel_smooth[i-window_samples:i, 0]
            ay_w = accel_smooth[i-window_samples:i, 1]
            az_w = accel_smooth[i-window_samples:i, 2]
            current_best_idx = get_dominant_axis_index(ax_w, ay_w, az_w, FS)

        dom_axis_array[i] = current_best_idx
        # Extract the exact data point from the chosen axis (subtract 1 because arrays are 0-indexed)
        chosen_raw_stitched[i] = accel_smooth[i, current_best_idx - 1]

    print("--- 4. Second Smoothing & Stroke Detection ---")
    # Apply a deeper smooth to the single chosen axis curve
    chosen_smooth = butter_lowpass_filter(chosen_raw_stitched, SECOND_CUTOFF, FS)

    # Calculate rolling min and max over the 10-second window
    stroke_window_sec = 10.0
    rolling_samples = int(stroke_window_sec * FS)
    rolling_min = pd.Series(chosen_smooth).rolling(window=rolling_samples, min_periods=1).min().values
    rolling_max = pd.Series(chosen_smooth).rolling(window=rolling_samples, min_periods=1).max().values

    # Calculate the lower 25% threshold line
    threshold_25 = rolling_min + 0.25 * (rolling_max - rolling_min)

    # --- NEW: Upward Crossing Cluster Logic ---
    
    # 1. Find EVERY single point where the signal crosses from below to above the 25% line
    raw_crossings = []
    for i in range(1, len(chosen_smooth)):
        if chosen_smooth[i-1] < threshold_25[i-1] and chosen_smooth[i] >= threshold_25[i]:
            raw_crossings.append(i)

    # 2. Filter the crossings: Group them if they happen within MIN_STROKE_TIME (0.9s)
    # and always pick the LAST crossing in that cluster as the true "Drive" start.
    crossing_indices = []
    crossing_times = []
    crossing_values = []

    if len(raw_crossings) > 0:
        current_cluster = [raw_crossings[0]]
        
        for i in range(1, len(raw_crossings)):
            idx = raw_crossings[i]
            prev_idx = current_cluster[-1]
            
            # If this crossing is less than MIN_STROKE_TIME from the previous one, it's the same noisy stroke
            if (time_sec[idx] - time_sec[prev_idx]) <= MIN_STROKE_TIME:
                current_cluster.append(idx)
            else:
                # We've moved on to a new stroke. Save the LAST crossing from the previous cluster.
                best_idx = current_cluster[-1]
                crossing_indices.append(best_idx)
                crossing_times.append(time_sec[best_idx])
                crossing_values.append(chosen_smooth[best_idx])
                
                # Start tracking the new cluster
                current_cluster = [idx]
                
        # Don't forget to save the very last cluster after the loop finishes!
        if current_cluster:
            best_idx = current_cluster[-1]
            crossing_indices.append(best_idx)
            crossing_times.append(time_sec[best_idx])
            crossing_values.append(chosen_smooth[best_idx])

            
    print("--- 5. Calculating SPM ---")
    raw_spm_times = []
    raw_spm_values = []
    
    for i in range(1, len(crossing_times)):
        dt = crossing_times[i] - crossing_times[i-1]
        spm = 60.0 / dt
        
        # Absolute sanity check (ignore physical impossibilities)
        if 10 <= spm <= 65:
            raw_spm_times.append(crossing_times[i])
            raw_spm_values.append(spm)

    # Convert to a Pandas Series for elegant filtering
    spm_series = pd.Series(raw_spm_values)

    # 1. Moving Median Filter (Window = 5 strokes)
    # This completely eliminates 1 or 2 stroke spikes, but accepts a new rate 
    # if it persists for the majority of the window.
    spm_median = spm_series.rolling(window=5, min_periods=1).median()

    # 2. Exponential Moving Average (EMA)
    # This lightly smooths the transitions so the line looks natural and fluid,
    # rather than looking like jagged stairs.
    spm_smoothed = spm_median.ewm(span=3, adjust=False).mean()

    # Extract final arrays for plotting
    spm_times = raw_spm_times
    spm_final_values = spm_smoothed.values


    print("--- 6. Plotting ---")
    fig, axs = plt.subplots(4, 1, figsize=(14, 14), sharex=True)

    # ---------- PLOT 1: Smoothed Original Acceleration ----------
    axs[0].plot(time_sec, accel_smooth[:,0], label='Accel X', alpha=0.8, color='#1f77b4')
    axs[0].plot(time_sec, accel_smooth[:,1], label='Accel Y', alpha=0.8, color='#ff7f0e')
    axs[0].plot(time_sec, accel_smooth[:,2], label='Accel Z', alpha=0.8, color='#2ca02c')
    axs[0].set_title(f'1. Lightly Smoothed Acceleration (Cutoff = {INIT_CUTOFF} Hz)')
    axs[0].set_ylabel('Accel (g)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True)

    # ---------- PLOT 2: Dominant Axis Tracker ----------
    axs[1].plot(time_sec, dom_axis_array, label='Selected Axis', color='purple', linewidth=2)
    axs[1].set_title('2. Dynamically Chosen Axis (1=X, 2=Y, 3=Z)')
    axs[1].set_ylabel('Axis Index')
    axs[1].set_yticks([1, 2, 3])
    axs[1].set_ylim(0.5, 3.5)
    axs[1].grid(True, axis='y')

    # ---------- PLOT 3: Chosen Data & Stroke Detection ----------
    axs[2].plot(time_sec, chosen_smooth, label=f'Chosen Axis (Smoothed {SECOND_CUTOFF} Hz)', color='black', linewidth=1.5)
    
    # Plot the new dynamic 25% threshold line
    axs[2].plot(time_sec, threshold_25, label='Lower 25% Min-Max Threshold', color='blue', linestyle='--', alpha=0.8)
    
    axs[2].scatter(crossing_times, crossing_values, color='green', s=50, zorder=5, label='Detected Stroke (Upward Crossing)')
    axs[2].set_title('3. Stroke Detection (Crossing Dynamic 25% Threshold)')
    axs[2].set_ylabel('Accel (g)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True)

    # ---------- PLOT 4: Stroke Rate ----------
    # Plot the raw SPM faintly in the background so you can see what the filter removed
    axs[3].step(raw_spm_times, raw_spm_values, where='post', color='gray', alpha=0.3, label='Raw SPM')
    
    # Plot the beautifully filtered SPM
    axs[3].plot(spm_times, spm_final_values, label='Filtered SPM (Median + EMA)', color='red', linewidth=2.5)
    
    axs[3].set_title('4. Calculated Stroke Rate')
    axs[3].set_xlabel('Time (seconds)')
    axs[3].set_ylabel('Strokes Per Minute (SPM)')
    axs[3].set_ylim(10, 60)
    axs[3].legend(loc='upper right')
    axs[3].grid(True)

    axs[0].set_xlim(time_sec[0], time_sec[-1])

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_rowing_data(CSV_FILE)