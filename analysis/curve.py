import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- CONFIGURATION ---
CSV_FILE = 'data/0423_1649.csv' 
FS = 100.0                           
PLOT_WINDOW_SEC = 20.0              
START_TIME_SEC = 100.0              # Skip the first X seconds of the file

# --- TUNING PARAMETERS ---
DOM_BUFFER_SIZE = int(20.0 * FS)    
MIN_MAX_BUFFER_SIZE = int(10.0 * FS)
MIN_STROKE_TIME_SEC = 0.9           

class EmaFilter:
    """Lightweight Exponential Moving Average filter."""
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.y = None

    def process(self, x):
        if self.y is None:
            self.y = x 
        else:
            self.y = (self.alpha * x) + ((1.0 - self.alpha) * self.y)
        return self.y


class LiveRowingProcessor:
    def __init__(self):
        # Raw Data Buffers
        self.t_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.x_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.y_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.z_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        
        # Processed Buffers
        self.dom_axis_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.chosen_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.threshold_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        
        # EMA Filters
        self.filterX = EmaFilter(alpha=0.3)
        self.filterY = EmaFilter(alpha=0.3)
        self.filterZ = EmaFilter(alpha=0.3)
        self.filterChosen = EmaFilter(alpha=0.2) 

        self.xBuf = deque(maxlen=DOM_BUFFER_SIZE)
        self.yBuf = deque(maxlen=DOM_BUFFER_SIZE)
        self.zBuf = deque(maxlen=DOM_BUFFER_SIZE)
        
        self.currentDomAxis = 1
        self.lastDomEvalTime = 0.0
        
        self.chosenBuf = deque(maxlen=MIN_MAX_BUFFER_SIZE)
        
        # Catch Detection State Machine
        self.history_buf = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.is_below_threshold = False
        self.temp_min_val = float('inf')
        self.temp_min_time = 0.0
        self.last_catch_time = None
        
        # Holds the normalized 0-100 arrays
        self.recent_strokes = deque(maxlen=5) 
        # Holds the detected finish index (0-100) for the latest strokes
        self.recent_finishes = deque(maxlen=5)
        
        self.crossings_t = deque(maxlen=50)
        self.crossings_val = deque(maxlen=50)
        
        # --- NEW: Ratio Tracking ---
        self.ratios_t = deque(maxlen=50)
        self.ratios_val = deque(maxlen=50)

    def _detect_finish_and_ratio(self, normalized_stroke, catch_time):
        """Finds the end of the parabolic drive and calculates the Rhythm Ratio"""
        # The peak of the drive should happen in the first half of the stroke
        peak_idx = np.argmax(normalized_stroke[:50])
        finish_idx = peak_idx
        
        # Look for the first local minimum after the peak (where the drop-off stops)
        for i in range(peak_idx, 90):
            if normalized_stroke[i] < normalized_stroke[i+1]:
                finish_idx = i
                break
        else:
            # Fallback: if no clear dip, find where the slope flattens out to near-zero
            for i in range(peak_idx, 90):
                slope = normalized_stroke[i+1] - normalized_stroke[i]
                if slope > -0.02: 
                    finish_idx = i
                    break
        
        # Ratio Calculation (Drive % / Recovery %)
        drive_pct = finish_idx
        recovery_pct = 100 - finish_idx
        
        if recovery_pct > 0:
            ratio = drive_pct / recovery_pct
            self.ratios_t.append(catch_time)
            self.ratios_val.append(ratio)
            
        return finish_idx

    def process_new_data(self, t_sec, ax, ay, az, speedKmph):
        self.t_plot.append(t_sec)
        
        # 1. Smoothing
        xs = self.filterX.process(ax)
        ys = self.filterY.process(ay)
        zs = self.filterZ.process(az)
        
        self.x_plot.append(xs)
        self.y_plot.append(ys)
        self.z_plot.append(zs)

        self.xBuf.append(xs)
        self.yBuf.append(ys)
        self.zBuf.append(zs)

        # 2. Track Dominant Axis
        if t_sec - self.lastDomEvalTime >= 1.0 and len(self.xBuf) >= 100:
            varX = np.var(self.xBuf)
            varY = np.var(self.yBuf)
            varZ = np.var(self.zBuf)

            if varX > varY and varX > varZ: self.currentDomAxis = 1
            elif varY > varX and varY > varZ: self.currentDomAxis = 2
            else: self.currentDomAxis = 3
            self.lastDomEvalTime = t_sec

        self.dom_axis_plot.append(self.currentDomAxis)

        # 3. Extract Chosen Axis
        chosenRaw = xs if self.currentDomAxis == 1 else (ys if self.currentDomAxis == 2 else zs)
        chosenSmooth = self.filterChosen.process(chosenRaw)

        self.chosenBuf.append(chosenSmooth)
        self.chosen_plot.append(chosenSmooth)
        self.history_buf.append((t_sec, chosenSmooth))

        # 4. Rolling Threshold
        rMin = min(self.chosenBuf)
        rMax = max(self.chosenBuf)
        threshold25 = rMin + 0.25 * (rMax - rMin)
        self.threshold_plot.append(threshold25)

        # 5. Catch Detection State Machine
        if len(self.chosenBuf) > 20: 
            if chosenSmooth < threshold25:
                if not self.is_below_threshold:
                    self.is_below_threshold = True
                    self.temp_min_val = chosenSmooth
                    self.temp_min_time = t_sec
                else:
                    if chosenSmooth < self.temp_min_val:
                        self.temp_min_val = chosenSmooth
                        self.temp_min_time = t_sec
            
            elif self.is_below_threshold:
                self.is_below_threshold = False
                
                if self.last_catch_time is None:
                    self.last_catch_time = self.temp_min_time
                elif (self.temp_min_time - self.last_catch_time) > MIN_STROKE_TIME_SEC:
                    stroke_data = [val for t, val in self.history_buf if self.last_catch_time <= t <= self.temp_min_time]
                    
                    if len(stroke_data) > 10:
                        # Normalize duration to 100 points
                        old_indices = np.linspace(0, 1, len(stroke_data))
                        new_indices = np.linspace(0, 1, 100)
                        normalized_stroke = np.interp(new_indices, old_indices, stroke_data)
                        
                        # Find the Release/Finish and Ratio
                        finish_idx = self._detect_finish_and_ratio(normalized_stroke, self.temp_min_time)
                        
                        self.recent_strokes.append(normalized_stroke)
                        self.recent_finishes.append(finish_idx)
                        
                        self.crossings_t.append(self.temp_min_time)
                        self.crossings_val.append(self.temp_min_val)

                    self.last_catch_time = self.temp_min_time

def stream_csv_data(filename, start_sec=0.0):
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
                
                if t_sec < start_sec: continue
                yield t_sec, x, y, z
            except ValueError: continue

# --- SETUP LIVE PLOTTING ---
processor = LiveRowingProcessor()
data_generator = stream_csv_data(CSV_FILE, START_TIME_SEC)

fig = plt.figure(figsize=(14, 18))
fig.canvas.manager.set_window_title('Advanced Stroke Phase Extractor')

# Custom Layout to share X axes properly
ax1 = plt.subplot(511)
ax2 = plt.subplot(512, sharex=ax1)
ax3 = plt.subplot(513, sharex=ax1)
ax4 = plt.subplot(514)             # Stroke Overlay uses 0-100%, so NO sharex
ax5 = plt.subplot(515, sharex=ax1) # Ratio shares Time X-axis
axs = [ax1, ax2, ax3, ax4, ax5]

# Plot 1, 2, 3 remains the same
line_x, = axs[0].plot([], [], label='EMA X', alpha=0.8, color='#1f77b4')
line_y, = axs[0].plot([], [], label='EMA Y', alpha=0.8, color='#ff7f0e')
line_z, = axs[0].plot([], [], label='EMA Z', alpha=0.8, color='#2ca02c')
axs[0].set_title('1. EMA Smoothed Acceleration')
axs[0].grid(True); axs[0].set_ylim(-3, 3)

line_dom, = axs[1].plot([], [], color='purple', linewidth=2)
axs[1].set_title('2. Dynamically Chosen Axis')
axs[1].set_yticks([1, 2, 3]); axs[1].set_ylim(0.5, 3.5); axs[1].grid(True, axis='y')

line_chosen, = axs[2].plot([], [], color='black', linewidth=1.5)
line_thresh, = axs[2].plot([], [], color='blue', linestyle='--', alpha=0.8)
scat_strokes = axs[2].scatter([], [], color='green', s=70, zorder=5, label='Catch')
axs[2].set_title('3. Stroke Detection (Catch Minimum)')
axs[2].grid(True); axs[2].set_ylim(-3, 3)

# --- CHANGED: Plot 4 (Overlay Comparison) ---
axs[3].set_title('4. Stroke Profile: Latest Stroke vs. Average of Past 4')
axs[3].set_xlabel('Stroke Cycle (%)')
axs[3].set_xlim(0, 100)
axs[3].grid(True)
line_avg_overlay, = axs[3].plot([], [], color='gray', linestyle='--', linewidth=2.5, label='Avg of Past 4')
line_latest_overlay, = axs[3].plot([], [], color='red', linewidth=3, label='Latest Stroke')
scat_finish = axs[3].scatter([], [], color='blue', s=80, zorder=5, label='Detected Finish/Release')
axs[3].legend(loc='upper right')

# --- NEW: Plot 5 (Rhythm Ratio) ---
line_ratio, = axs[4].plot([], [], color='teal', linewidth=2.5, marker='o')
axs[4].set_title('5. Rhythm Ratio (Drive Time / Recovery Time)')
axs[4].set_xlabel('Time (seconds)')
axs[4].set_ylabel('Ratio (e.g. 0.5 = 1:2)')
axs[4].grid(True)
axs[4].set_ylim(0.2, 1.2) # Typical rowing ratios fit in here

def update(frame):
    CHUNK_SIZE = 9  
    latest_t = 0
    for _ in range(CHUNK_SIZE):
        try:
            t, x, y, z = next(data_generator)
            processor.process_new_data(t, x, y, z, 5.0) 
            latest_t = t
        except StopIteration:
            return line_x, line_y, line_z, line_dom, line_chosen, line_thresh, scat_strokes, line_avg_overlay, line_latest_overlay, scat_finish, line_ratio

    if len(processor.t_plot) == 0:
        return line_x,
        
    line_x.set_data(processor.t_plot, processor.x_plot)
    line_y.set_data(processor.t_plot, processor.y_plot)
    line_z.set_data(processor.t_plot, processor.z_plot)
    line_dom.set_data(processor.t_plot, processor.dom_axis_plot)
    line_chosen.set_data(processor.t_plot, processor.chosen_plot)
    line_thresh.set_data(processor.t_plot, processor.threshold_plot)
    
    if len(processor.crossings_t) > 0:
        offsets = np.column_stack((processor.crossings_t, processor.crossings_val))
        scat_strokes.set_offsets(offsets)
        
    current_t = processor.t_plot[-1]
    start_t = processor.t_plot[0] 
    min_x = max(start_t, current_t - PLOT_WINDOW_SEC)
    max_x = max(start_t + PLOT_WINDOW_SEC, current_t)
    ax1.set_xlim(min_x, max_x)

    # --- Update Overlay (Plot 4) ---
    x_norm = np.linspace(0, 100, 100)
    strokes_list = list(processor.recent_strokes)
    finish_list = list(processor.recent_finishes)
    
    if len(strokes_list) > 0:
        latest_stroke = strokes_list[-1]
        latest_finish_idx = finish_list[-1]
        
        # Plot the average of the *previous* 4 strokes
        if len(strokes_list) > 1:
            avg_past = np.mean(strokes_list[:-1], axis=0)
            line_avg_overlay.set_data(x_norm, avg_past)
        else:
            line_avg_overlay.set_data(x_norm, latest_stroke)
            
        # Plot the latest stroke in bold red
        line_latest_overlay.set_data(x_norm, latest_stroke)
        
        # Plot the blue dot exactly at the detected finish line
        scat_finish.set_offsets(np.column_stack(([latest_finish_idx], [latest_stroke[latest_finish_idx]])))
        
        ymin = np.min(strokes_list) - 0.5
        ymax = np.max(strokes_list) + 0.5
        axs[3].set_ylim(ymin, ymax)
        
    # --- Update Ratio Plot (Plot 5) ---
    if len(processor.ratios_t) > 0:
        line_ratio.set_data(processor.ratios_t, processor.ratios_val)
        # Dynamically scale Y limits based on user's actual ratios
        ax5.set_ylim(min(0.3, min(processor.ratios_val)-0.1), max(1.0, max(processor.ratios_val)+0.1))

    return line_x, line_y, line_z, line_dom, line_chosen, line_thresh, scat_strokes, line_avg_overlay, line_latest_overlay, scat_finish, line_ratio

ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()