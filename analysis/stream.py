import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt
from scipy.fft import rfft, rfftfreq
from collections import deque

# --- CONFIGURATION ---
CSV_FILE = 'data/0423_1649.csv'
FS = 50.0                           
CHUNK_SIZE = 15                      # Number of rows to read per animation frame (controls playback speed)
PLOT_WINDOW_SEC = 30.0              # How many seconds of data to show on screen

# --- TUNING PARAMETERS ---
INIT_CUTOFF = 10.0                  
SECOND_CUTOFF = 10.0                
WINDOW_SEC = 20.0                   
MIN_STROKE_TIME = 0.9               
STROKE_WINDOW_SEC = 10.0            

# Buffer sizes based on sampling rate
MAX_BUFFER = int(PLOT_WINDOW_SEC * FS)
DOM_EVAL_BUFFER = int(WINDOW_SEC * FS)
MIN_MAX_BUFFER = int(STROKE_WINDOW_SEC * FS)


def butter_lowpass_filter(data, cutoff, fs, order=4):
    if len(data) <= 15: # filtfilt needs padding room
        return data
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def get_dominant_axis_index(w_x, w_y, w_z, fs):
    axes = {1: w_x, 2: w_y, 3: w_z}
    best_idx = 1
    best_score = -1.0
    
    for idx, data in axes.items():
        detrended = data - np.mean(data)
        n = len(detrended)
        if n == 0: continue
        
        yf = np.abs(rfft(detrended))
        xf = rfftfreq(n, 1 / fs)

        valid_idx = np.where((xf >= 0.25) & (xf <= 1.0))[0]
        if len(valid_idx) == 0: continue

        dominant_peak_val = np.max(yf[valid_idx])
        total_energy = np.sum(yf)
        score = dominant_peak_val / (total_energy + 1e-6)

        if score > best_score:
            best_score = score
            best_idx = idx

    return best_idx


class LiveRowingProcessor:
    """State machine that holds our live rolling buffers and logic."""
    def __init__(self):
        # Raw Data Buffers
        self.t = deque(maxlen=MAX_BUFFER)
        self.x = deque(maxlen=MAX_BUFFER)
        self.y = deque(maxlen=MAX_BUFFER)
        self.z = deque(maxlen=MAX_BUFFER)
        
        # Processed Data Buffers
        self.dom_axis = deque(maxlen=MAX_BUFFER)
        self.chosen_smooth = deque(maxlen=MAX_BUFFER)
        self.threshold_25 = deque(maxlen=MAX_BUFFER)
        
        # Stroke Tracking
        self.crossings_t = deque(maxlen=50)
        self.crossings_val = deque(maxlen=50)
        
        # SPM Tracking
        self.raw_spm_t = deque(maxlen=50)
        self.raw_spm_val = deque(maxlen=50)
        self.spm_t = deque(maxlen=50)
        self.spm_val = deque(maxlen=50)
        
        # Live State Variables
        self.current_dom_idx = 1
        self.last_eval_time = 0.0
        
        # Clustering State Machine
        self.pending_stroke_t = None
        self.pending_stroke_val = None
        self.last_finalized_stroke_t = None
        
        # Filters State
        self.spm_history = deque(maxlen=5) # For rolling median
        self.current_ema = None
        
    def process_new_data(self, t_new, x_new, y_new, z_new):
        self.t.append(t_new)
        self.x.append(x_new)
        self.y.append(y_new)
        self.z.append(z_new)
        
        # Need enough data to start evaluating
        if len(self.t) < MIN_MAX_BUFFER:
            return
            
        # 1. Smooth the current visible window (Pseudo-live filtering)
        x_smooth = butter_lowpass_filter(np.array(self.x), INIT_CUTOFF, FS)
        y_smooth = butter_lowpass_filter(np.array(self.y), INIT_CUTOFF, FS)
        z_smooth = butter_lowpass_filter(np.array(self.z), INIT_CUTOFF, FS)
        
        # 2. Track Dominant Axis (Evaluate every 1 second)
        if t_new - self.last_eval_time >= 1.0 and len(x_smooth) >= DOM_EVAL_BUFFER:
            self.current_dom_idx = get_dominant_axis_index(
                x_smooth[-DOM_EVAL_BUFFER:], 
                y_smooth[-DOM_EVAL_BUFFER:], 
                z_smooth[-DOM_EVAL_BUFFER:], FS)
            self.last_eval_time = t_new
            
        self.dom_axis.append(self.current_dom_idx)
        
        # 3. Extract the chosen axis and apply second smoothing
        # Note: In a strict live environment, you'd filter just the chosen stream. 
        # Here we extract the latest point from the smoothed array.
        latest_chosen_val = [x_smooth[-1], y_smooth[-1], z_smooth[-1]][self.current_dom_idx - 1]
        
        # Apply the deeper 10Hz filter just to the trailing window of chosen data
        temp_chosen_buffer = list(self.chosen_smooth) + [latest_chosen_val]
        if len(temp_chosen_buffer) > 15:
            deep_smooth = butter_lowpass_filter(np.array(temp_chosen_buffer), SECOND_CUTOFF, FS)
            current_deep_val = deep_smooth[-1]
        else:
            current_deep_val = latest_chosen_val
            
        self.chosen_smooth.append(current_deep_val)
        
        # 4. Rolling Min/Max & Dynamic Threshold (over last 10 seconds)
        recent_chosen = np.array(self.chosen_smooth)[-MIN_MAX_BUFFER:]
        r_min, r_max = np.min(recent_chosen), np.max(recent_chosen)
        thresh = r_min + 0.25 * (r_max - r_min)
        self.threshold_25.append(thresh)
        
        # 5. Live Stroke Detection (State Machine)
        if len(self.chosen_smooth) >= 2:
            prev_val = self.chosen_smooth[-2]
            curr_val = self.chosen_smooth[-1]
            prev_thresh = self.threshold_25[-2]
            curr_thresh = self.threshold_25[-1]
            
            # Did we just cross upward?
            if prev_val < prev_thresh and curr_val >= curr_thresh:
                # We have a crossing!
                if self.pending_stroke_t is None:
                    # Brand new cluster
                    self.pending_stroke_t = t_new
                    self.pending_stroke_val = curr_val
                else:
                    # Check if it's within the bounce window
                    if (t_new - self.pending_stroke_t) <= MIN_STROKE_TIME:
                        # Update pending stroke to this newer one
                        self.pending_stroke_t = t_new
                        self.pending_stroke_val = curr_val
                    else:
                        # The old pending stroke is finalized! Time to record it.
                        self._finalize_stroke(self.pending_stroke_t, self.pending_stroke_val)
                        # Start a new pending stroke
                        self.pending_stroke_t = t_new
                        self.pending_stroke_val = curr_val
                        
            # Edge case logic: If it's been longer than MIN_STROKE_TIME since our pending stroke,
            # and no new crossing happened, finalize the pending stroke immediately.
            elif self.pending_stroke_t is not None and (t_new - self.pending_stroke_t) > MIN_STROKE_TIME:
                self._finalize_stroke(self.pending_stroke_t, self.pending_stroke_val)
                self.pending_stroke_t = None

    def _finalize_stroke(self, stroke_time, stroke_val):
        self.crossings_t.append(stroke_time)
        self.crossings_val.append(stroke_val)
        
        if self.last_finalized_stroke_t is not None:
            dt = stroke_time - self.last_finalized_stroke_t
            spm = 60.0 / dt
            
            if 10 <= spm <= 65:
                self.raw_spm_t.append(stroke_time)
                self.raw_spm_val.append(spm)
                
                # Filtering live SPM
                self.spm_history.append(spm)
                spm_median = np.median(self.spm_history)
                
                # EMA
                alpha = 2.0 / (3 + 1) # Span = 3
                if self.current_ema is None:
                    self.current_ema = spm_median
                else:
                    self.current_ema = (spm_median * alpha) + (self.current_ema * (1 - alpha))
                    
                self.spm_t.append(stroke_time)
                self.spm_val.append(self.current_ema)
                
        self.last_finalized_stroke_t = stroke_time

def stream_csv_data(filename):
    """Generator to read the CSV line by line like a live feed."""
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        first_t = None
        for row in reader:
            if not row or row[0] != 'IMU' or len(row) < 8: continue
            try:
                t_ms = int(row[1])
                x = float(row[2]) / 100.0
                y = float(row[3]) / 100.0
                z = float(row[4]) / 100.0
                
                if first_t is None: first_t = t_ms
                t_sec = (t_ms - first_t) / 1000.0
                yield t_sec, x, y, z
            except ValueError:
                continue

# --- SETUP LIVE PLOTTING ---
processor = LiveRowingProcessor()
data_generator = stream_csv_data(CSV_FILE)

fig, axs = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
fig.canvas.manager.set_window_title('Live Rowing Telemetry')

# Plot 1 Init
line_x, = axs[0].plot([], [], label='Accel X', alpha=0.8, color='#1f77b4')
line_y, = axs[0].plot([], [], label='Accel Y', alpha=0.8, color='#ff7f0e')
line_z, = axs[0].plot([], [], label='Accel Z', alpha=0.8, color='#2ca02c')
axs[0].set_title(f'1. Lightly Smoothed Acceleration')
axs[0].set_ylabel('Accel (g)')
axs[0].legend(loc='upper right')
axs[0].grid(True)
axs[0].set_ylim(-3, 3) # Static y-limits help visualization

# Plot 2 Init
line_dom, = axs[1].plot([], [], label='Selected Axis', color='purple', linewidth=2)
axs[1].set_title('2. Dynamically Chosen Axis')
axs[1].set_ylabel('Axis Index')
axs[1].set_yticks([1, 2, 3])
axs[1].set_ylim(0.5, 3.5)
axs[1].grid(True, axis='y')

# Plot 3 Init
line_chosen, = axs[2].plot([], [], label='Chosen Axis', color='black', linewidth=1.5)
line_thresh, = axs[2].plot([], [], label='Lower 25% Threshold', color='blue', linestyle='--', alpha=0.8)
scat_strokes = axs[2].scatter([], [], color='green', s=50, zorder=5, label='Detected Stroke')
axs[2].set_title('3. Stroke Detection')
axs[2].set_ylabel('Accel (g)')
axs[2].legend(loc='upper right')
axs[2].grid(True)
axs[2].set_ylim(-3, 3)

# Plot 4 Init
line_raw_spm, = axs[3].step([], [], where='post', color='gray', alpha=0.3, label='Raw SPM')
line_spm, = axs[3].plot([], [], label='Filtered SPM', color='red', linewidth=2.5)
axs[3].set_title('4. Calculated Stroke Rate')
axs[3].set_xlabel('Time (seconds)')
axs[3].set_ylabel('SPM')
axs[3].set_ylim(10, 60)
axs[3].legend(loc='upper right')
axs[3].grid(True)

def update(frame):
    # Pull a chunk of data to process
    latest_t = 0
    for _ in range(CHUNK_SIZE):
        try:
            t, x, y, z = next(data_generator)
            processor.process_new_data(t, x, y, z)
            latest_t = t
        except StopIteration:
            # End of file reached
            return line_x, line_y, line_z, line_dom, line_chosen, line_thresh, scat_strokes, line_raw_spm, line_spm

    if len(processor.t) == 0:
        return line_x,
        
    # --- Update Plots ---
    # Apply rolling window smoothing purely for visualization
    x_viz = butter_lowpass_filter(np.array(processor.x), INIT_CUTOFF, FS)
    y_viz = butter_lowpass_filter(np.array(processor.y), INIT_CUTOFF, FS)
    z_viz = butter_lowpass_filter(np.array(processor.z), INIT_CUTOFF, FS)
    
    line_x.set_data(processor.t, x_viz)
    line_y.set_data(processor.t, y_viz)
    line_z.set_data(processor.t, z_viz)
    
    # Slice the time array to match the delayed start variables
    t_dom = list(processor.t)[-len(processor.dom_axis):]
    line_dom.set_data(t_dom, processor.dom_axis)
    
    t_chosen = list(processor.t)[-len(processor.chosen_smooth):]
    line_chosen.set_data(t_chosen, processor.chosen_smooth)
    line_thresh.set_data(t_chosen, processor.threshold_25)
    
    # Scatter plot needs 2D array for offsets
    if len(processor.crossings_t) > 0:
        offsets = np.column_stack((processor.crossings_t, processor.crossings_val))
        scat_strokes.set_offsets(offsets)
        
    line_raw_spm.set_data(processor.raw_spm_t, processor.raw_spm_val)
    line_spm.set_data(processor.spm_t, processor.spm_val)

    # Dynamic X-Axis scrolling
    current_t = processor.t[-1]
    min_x = max(0, current_t - PLOT_WINDOW_SEC)
    max_x = max(PLOT_WINDOW_SEC, current_t)
    
    for ax in axs:
        ax.set_xlim(min_x, max_x)

    return line_x, line_y, line_z, line_dom, line_chosen, line_thresh, scat_strokes, line_raw_spm, line_spm

# Interval is ms between frames. With CHUNK_SIZE=5 at 50Hz, real-time is 100ms.
# Set it lower (e.g., 20) to watch the simulation run faster than real-time.
ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

plt.tight_layout()
plt.show()