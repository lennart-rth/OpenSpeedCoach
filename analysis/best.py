import csv
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# --- CONFIGURATION ---
CSV_FILE = 'data/athome.csv'
TARGET_FS = 50.0          
WINDOW_SEC = 6.0      
STEP_SEC = 1          

# --- PRE-SMOOTHING (SIGNAL LEVEL) ---
SIGNAL_SMOOTHING_ALPHA = 0.3 

# --- STATISTICAL FILTER TUNING (METRIC LEVEL) ---
CLEAN_MAX_SPM = 55.0     
DEAD_REC_THRESH = 5  
SPEED_GATE = 1.5      
HISTORY_SIZE = 4       

# --- ACTIVATION DELAY ---
STRIKE_CONFIRMATION_THRESHOLD = 3  # Must see 3 valid windows before showing SPM

# --- CUTOFFS ---
MAX_ROWING_SPM_CUTOFF = 55 
MIN_ROWING_SPM_CUTOFF = 10
ALLOWED_DEVIATION = 7.0  
SMOOTHING_ALPHA = 0.3      

def analyze_robust_spm(filename):
    imu_ts, accel_mags = [], []
    gps_ts, gps_lats, gps_lons, gps_speeds = [], [], [], []

    print(f"--- 1. Loading and Pre-Processing ---")
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row: continue
                if row[0] == 'IMU':
                    try:
                        ts = int(row[1])
                        mag = np.sqrt((float(row[2])/100)**2 + (float(row[3])/100)**2 + (float(row[4])/100)**2) - 9.81
                        imu_ts.append(ts); accel_mags.append(mag)
                    except: continue
                elif row[0] == 'GPS':
                    try:
                        gps_ts.append(int(row[1])); gps_lats.append(float(row[2]))
                        gps_lons.append(float(row[3])); gps_speeds.append(float(row[4]))
                    except: continue
    except FileNotFoundError:
        print("File not found.")
        return

    imu_ts, accel_mags = np.array(imu_ts), np.array(accel_mags)
    t_raw = (imu_ts - imu_ts[0]) / 1000.0
    t_uniform = np.arange(0, t_raw[-1], 1.0 / TARGET_FS)
    mags_uniform = np.interp(t_uniform, t_raw, accel_mags)

    mags_smoothed = np.zeros_like(mags_uniform)
    curr = mags_uniform[0]
    for i in range(len(mags_uniform)):
        curr = (SIGNAL_SMOOTHING_ALPHA * mags_uniform[i]) + ((1 - SIGNAL_SMOOTHING_ALPHA) * curr)
        mags_smoothed[i] = curr
    
    moving_mask = np.zeros_like(t_uniform, dtype=bool)
    if len(gps_ts) > 1:
        last_lat, last_lon = gps_lats[0], gps_lons[0]
        for i in range(1, len(gps_ts)):
            dy = (gps_lats[i] - last_lat) * 111111
            dx = (gps_lons[i] - last_lon) * 111111 * np.cos(np.radians(last_lat))
            d = np.sqrt(dx**2 + dy**2)
            if (d > DEAD_REC_THRESH) or (gps_speeds[i] > SPEED_GATE):
                t_start, t_end = (gps_ts[i-1]-imu_ts[0])/1000.0, (gps_ts[i]-imu_ts[0])/1000.0
                moving_mask[(t_uniform >= t_start) & (t_uniform <= t_end)] = True
                last_lat, last_lon = gps_lats[i], gps_lons[i]

    print(f"--- 2. Processing with Confirmation Logic ---")
    window_samples = int(WINDOW_SEC * TARGET_FS)
    step_samples = int(STEP_SEC * TARGET_FS)
    min_lag, max_lag = int(TARGET_FS * (60.0/100.0)), int(TARGET_FS * (60.0/15.0))

    raw_detections = [] 
    final_spm_times, final_spm_values = [], []
    history = deque(maxlen=HISTORY_SIZE)
    
    # NEW: Confirmation State
    confirmation_count = 0

    for i in range(0, len(mags_smoothed) - window_samples, step_samples):
        current_t = t_uniform[i + window_samples//2]
        is_moving = moving_mask[i + window_samples//2]
        
        if not is_moving:
            current_raw_spm = 0.0
        else:
            window = mags_smoothed[i : i + window_samples]
            window = window - np.mean(window)
            autocorr = np.correlate(window, window, mode='full')
            autocorr = autocorr[len(autocorr)//2:]
            valid_lags = autocorr[min_lag:max_lag]
            best_lag = np.argmax(valid_lags) + min_lag
            current_raw_spm = 60.0 / (best_lag / TARGET_FS)
            
            if autocorr[best_lag] <= np.mean(autocorr[min_lag:max_lag]) * 1.2:
                current_raw_spm = 0.0

        # Hard Cutoffs
        if current_raw_spm > MAX_ROWING_SPM_CUTOFF or current_raw_spm < MIN_ROWING_SPM_CUTOFF:
            current_raw_spm = 0.0 

        # --- NEW: CONFIRMATION LOGIC ---
        if current_raw_spm > 0:
            confirmation_count += 1
        else:
            confirmation_count = 0 # Reset immediately if rhythm is lost

        # Only accept the stroke rate if we've seen enough consistent windows
        if confirmation_count < STRIKE_CONFIRMATION_THRESHOLD:
            accepted_spm = 0.0
        else:
            # Statistical Filter (History Based)
            if len(history) < HISTORY_SIZE:
                accepted_spm = current_raw_spm
                history.append(accepted_spm)
            else:
                avg_history = sum(history) / len(history)
                if abs(current_raw_spm - avg_history) <= ALLOWED_DEVIATION:
                    accepted_spm = current_raw_spm
                    history.append(accepted_spm)
                else:
                    accepted_spm = avg_history 

        if accepted_spm > 0:
            raw_detections.append(accepted_spm)
            
        final_spm_times.append(current_t)
        final_spm_values.append(accepted_spm)

    # 4. Final Smoothing
    smooth_spm = []
    if final_spm_values:
        smooth_spm.append(final_spm_values[0])
        for val in final_spm_values[1:]:
            alpha = 1.0 if (val == 0 or smooth_spm[-1] == 0) else SMOOTHING_ALPHA
            smooth_spm.append(alpha * val + (1 - alpha) * smooth_spm[-1])

    # --- PLOTTING ---
    plt.figure(figsize=(12, 12))

    # Signal detail
    plt.subplot(3, 1, 1)
    chunk = (t_uniform > 100) & (t_uniform < 140)
    plt.plot(t_uniform[chunk], mags_uniform[chunk], color='red', alpha=0.3, label='Raw')
    plt.plot(t_uniform[chunk], mags_smoothed[chunk], color='blue', label='Smoothed')
    plt.title('IMU Signal Quality')
    plt.legend(); plt.grid(True)

    # SPM Plot
    plt.subplot(3, 1, 2)
    plt.fill_between(t_uniform, 0, 60, where=moving_mask, color='green', alpha=0.15, label='GPS: Moving')
    plt.fill_between(t_uniform, 0, 60, where=~moving_mask, color='red', alpha=0.15, label='GPS: Stationary')
    plt.plot(final_spm_times, smooth_spm, 'g-', linewidth=2, label='Confirmed SPM')
    plt.title(f'Stroke Rate (Min 3 strokes to activate)')
    plt.ylim(0, 60); plt.legend(); plt.grid(True, alpha=0.3)

    # Histogram
    plt.subplot(3, 1, 3)
    if raw_detections:
        plt.hist(raw_detections, bins=np.arange(10, 60, 1), color='orange', edgecolor='black')
        plt.title('Confirmed SPM Distribution')
        plt.grid(axis='y', alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_robust_spm(CSV_FILE)