import csv
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# --- CONFIGURATION ---
CSV_FILE = 'data/04161551.csv'  # REPLACE with your filename
FS = 50                    # Sampling Frequency (Hz)

# ==========================================
# PART 1: Helper Classes (Filter & Detector)
# ==========================================

class LowPassFilter:
    """
    A simple Alpha-Filter (Exponential Moving Average) 
    replacing Scipy's butter/sosfilt.
    """
    def __init__(self, cutoff, fs):
        # Calculate alpha based on the desired cutoff frequency
        # Time constant tau = 1 / (2 * pi * cutoff)
        # alpha = dt / (tau + dt)
        dt = 1.0 / fs
        tau = 1.0 / (2 * np.pi * cutoff)
        self.alpha = dt / (tau + dt)
        self.last_y = 0.0
        
    def step(self, x):
        # Y[n] = alpha * X[n] + (1 - alpha) * Y[n-1]
        y = self.alpha * x + (1 - self.alpha) * self.last_y
        self.last_y = y
        return y

class StrokeDetector:
    def __init__(self, fs, min_spm=15, max_spm=60):
        self.min_interval_ms = (60.0 / max_spm) * 1000
        self.last_peak_time = -10000
        self.spm_buffer = deque(maxlen=3)
        self.current_spm = 0.0
        self.val_buffer = deque(maxlen=int(2.0*fs)) 
        
    def update(self, timestamp, value):
        self.val_buffer.append(value)
        if len(self.val_buffer) < 3: return 0.0
        
        y_prev = self.val_buffer[-2]
        y_now = self.val_buffer[-1]
        y_old = self.val_buffer[-3]
        
        if (y_prev > y_old) and (y_prev > y_now): # Local Max
            if (timestamp - self.last_peak_time) > self.min_interval_ms:
                threshold = np.mean(list(self.val_buffer)) + 0.5 * np.std(list(self.val_buffer))
                if y_prev > threshold:
                    if self.last_peak_time > 0:
                        delta_ms = timestamp - self.last_peak_time
                        if delta_ms > 0:
                            inst_spm = 60000.0 / delta_ms
                            if 10 < inst_spm < 100:
                                self.spm_buffer.append(inst_spm)
                                self.current_spm = np.mean(self.spm_buffer)
                    self.last_peak_time = timestamp
        return self.current_spm

# ==========================================
# PART 2: The Advanced Algorithm (PCA)
# ==========================================

def run_advanced_analysis(filename):
    lpf = LowPassFilter(cutoff=3.0, fs=FS)
    detector = StrokeDetector(fs=FS)
    
    # PCA State
    buffer_size = 5 * FS # 5 Seconds calibration window
    acc_buffer = deque(maxlen=buffer_size)
    pca_vector = np.array([1.0, 0.0, 0.0]) # Default X
    samples_since_pca = 0
    
    timestamps = []
    spm_values = []
    
    print(f"--- Running ADVANCED Method on {filename} ---")
    
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0] != 'IMU': continue
                
                try:
                    ts = int(row[1])
                    ax = float(row[2]) / 100.0
                    ay = float(row[3]) / 100.0
                    az = float(row[4]) / 100.0
                    
                    raw_vec = np.array([ax, ay, az])
                    
                    # --- CORE LOGIC ---
                    # 1. Update Buffer & Recalculate PCA periodically
                    acc_buffer.append(raw_vec)
                    samples_since_pca += 1
                    
                    if len(acc_buffer) == buffer_size and samples_since_pca >= FS:
                        # Perform PCA
                        data = np.array(acc_buffer)
                        centered = data - np.mean(data, axis=0)
                        cov = np.cov(centered, rowvar=False)
                        eig_vals, eig_vecs = np.linalg.eigh(cov)
                        # Max variance vector (Surge axis)
                        pca_vector = eig_vecs[:, -1]
                        samples_since_pca = 0
                    
                    # 2. Project onto Virtual Surge Axis
                    surge_accel = np.dot(raw_vec, pca_vector)
                    
                    # 3. Filter & Detect
                    filt_val = lpf.step(surge_accel)
                    spm = detector.update(ts, filt_val)
                    
                    timestamps.append(ts)
                    spm_values.append(spm)
                    
                except (ValueError, IndexError): continue
    except FileNotFoundError:
        print("File not found.")
        return

    # Plot
    if timestamps:
        t_sec = (np.array(timestamps) - timestamps[0]) / 1000.0
        plt.figure(figsize=(10, 5))
        plt.plot(t_sec, spm_values, label='Advanced PCA', color='orange')
        plt.title('Stroke Rate (Advanced Method)')
        plt.xlabel('Time (s)')
        plt.ylabel('SPM')
        plt.grid(True)
        plt.legend()
        plt.savefig('graph_advanced.png')
        print("Graph saved as graph_advanced.png")
        plt.show()

if __name__ == "__main__":
    run_advanced_analysis(CSV_FILE)
