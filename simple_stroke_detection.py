import csv
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from scipy.signal import butter, sosfilt

# --- CONFIGURATION ---
CSV_FILE = '10070830.csv'  # REPLACE with your filename
FS = 50                    # Sampling Frequency (Hz)

# ==========================================
# PART 1: Helper Classes (Filter & Detector)
# ==========================================

class LowPassFilter:
    def __init__(self, cutoff, fs, order=2):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.sos = butter(order, normal_cutoff, btype='low', analog=False, output='sos')
        self.z = np.zeros((self.sos.shape[0], 2))
        
    def step(self, x):
        y, self.z = sosfilt(self.sos, [x], zi=self.z)
        return y[0]

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
                # Dynamic Threshold: Mean + 0.5 * StdDev
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
# PART 2: The Simple Algorithm
# ==========================================

def run_simple_analysis(filename):
    lpf = LowPassFilter(cutoff=3.0, fs=FS)
    detector = StrokeDetector(fs=FS)
    
    timestamps = []
    spm_values = []
    
    print(f"--- Running SIMPLE Method on {filename} ---")
    
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
                    
                    # --- CORE LOGIC ---
                    # 1. Magnitude
                    mag = np.sqrt(ax**2 + ay**2 + az**2)
                    # 2. Remove Gravity (approx)
                    mag -= 9.81
                    # 3. Filter
                    filt_mag = lpf.step(mag)
                    # 4. Detect
                    spm = detector.update(ts, filt_mag)
                    
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
        plt.plot(t_sec, spm_values, label='Simple Magnitude', color='blue')
        plt.title('Stroke Rate (Simple Method)')
        plt.xlabel('Time (s)')
        plt.ylabel('SPM')
        plt.grid(True)
        plt.legend()
        plt.savefig('graph_simple.png')
        print("Graph saved as graph_simple.png")
        plt.show()

if __name__ == "__main__":
    run_simple_analysis(CSV_FILE)
