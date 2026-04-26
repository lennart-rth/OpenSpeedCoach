import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- CONFIGURATION ---
CSV_FILE = 'data/0423_1649.csv' # Make sure this CSV is actually recorded at 100Hz!
FS = 100.0                           
PLOT_WINDOW_SEC = 20.0              

# --- TUNING PARAMETERS ---
DOM_BUFFER_SIZE = int(20.0 * FS)    # 2000 samples
MIN_MAX_BUFFER_SIZE = int(10.0 * FS)# 1000 samples
MIN_STROKE_TIME_SEC = 0.9           

# --- NEW: EMA FILTER ---
class EmaFilter:
    """
    Lightweight Exponential Moving Average filter.
    Alpha = 0.2 to 0.3 at 100Hz provides a beautiful, smooth curve 
    for rowing stroke detection without Biquad overshoot.
    """
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.y = None

    def process(self, x):
        if self.y is None:
            self.y = x # Seed the filter on the first sample
        else:
            self.y = (self.alpha * x) + ((1.0 - self.alpha) * self.y)
        return self.y


class LiveRowingProcessor:
    def __init__(self):
        # Raw Data Buffers for plotting
        self.t_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.x_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.y_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.z_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        
        # Processed Buffers for plotting
        self.dom_axis_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.chosen_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        self.threshold_plot = deque(maxlen=int(PLOT_WINDOW_SEC * FS))
        
        # Now using EMA Filters
        self.filterX = EmaFilter(alpha=0.3)
        self.filterY = EmaFilter(alpha=0.3)
        self.filterZ = EmaFilter(alpha=0.3)
        
        # We apply a second EMA to the chosen axis for extra smoothing before min/max bounds
        self.filterChosen = EmaFilter(alpha=0.2) 

        self.xBuf = deque(maxlen=DOM_BUFFER_SIZE)
        self.yBuf = deque(maxlen=DOM_BUFFER_SIZE)
        self.zBuf = deque(maxlen=DOM_BUFFER_SIZE)
        
        self.currentDomAxis = 1
        self.lastDomEvalTime = 0.0
        
        self.chosenBuf = deque(maxlen=MIN_MAX_BUFFER_SIZE)
        
        self.pendingStrokeTime = None
        self.pendingStrokeVal = None
        self.lastFinalizedStrokeTime = None
        
        self.prevChosenSmooth = 0.0
        self.prevThreshold = 0.0

        # CHANGED: Window reduced to 3 for faster pace transitions
        self.spmHistory = deque(maxlen=3)
        self.currentEma = None

        # Output Data for Plots
        self.crossings_t = deque(maxlen=50)
        self.crossings_val = deque(maxlen=50)
        self.raw_spm_t = deque(maxlen=50)
        self.raw_spm_val = deque(maxlen=50)
        self.spm_t = deque(maxlen=50)
        self.spm_val = deque(maxlen=50)

    def _finalize_stroke(self, stroke_time):
        if self.lastFinalizedStrokeTime is not None:
            dt = stroke_time - self.lastFinalizedStrokeTime
            rawSpm = 60.0 / dt

            # Sanity Check
            if 10.0 <= rawSpm <= 65.0:
                self.raw_spm_t.append(stroke_time)
                self.raw_spm_val.append(rawSpm)

                self.spmHistory.append(rawSpm)
                medianSpm = np.median(self.spmHistory)

                # CHANGED: Fast EMA (80% New Pace, 20% Old Pace)
                if self.currentEma is None:
                    self.currentEma = medianSpm
                else:
                    self.currentEma = (medianSpm * 0.8) + (self.currentEma * 0.2)

                self.spm_t.append(stroke_time)
                self.spm_val.append(self.currentEma)

        self.lastFinalizedStrokeTime = stroke_time

    def process_new_data(self, t_sec, ax, ay, az, speedKmph):
        self.t_plot.append(t_sec)
        
        # 1. Initial 100Hz EMA Smoothing
        xs = self.filterX.process(ax)
        ys = self.filterY.process(ay)
        zs = self.filterZ.process(az)
        
        self.x_plot.append(xs)
        self.y_plot.append(ys)
        self.z_plot.append(zs)

        self.xBuf.append(xs)
        self.yBuf.append(ys)
        self.zBuf.append(zs)

        # 2. Track Dominant Axis (Every 1 second)
        if t_sec - self.lastDomEvalTime >= 1.0 and len(self.xBuf) >= 100:
            varX = np.var(self.xBuf)
            varY = np.var(self.yBuf)
            varZ = np.var(self.zBuf)

            if varX > varY and varX > varZ:
                self.currentDomAxis = 1
            elif varY > varX and varY > varZ:
                self.currentDomAxis = 2
            else:
                self.currentDomAxis = 3
                
            self.lastDomEvalTime = t_sec

        self.dom_axis_plot.append(self.currentDomAxis)

        # 3. Extract Chosen & Second EMA Smoothing
        chosenRaw = xs if self.currentDomAxis == 1 else (ys if self.currentDomAxis == 2 else zs)
        chosenSmooth = self.filterChosen.process(chosenRaw)

        self.chosenBuf.append(chosenSmooth)
        self.chosen_plot.append(chosenSmooth)

        # 4. Rolling Min/Max
        rMin = min(self.chosenBuf)
        rMax = max(self.chosenBuf)
        threshold25 = rMin + 0.25 * (rMax - rMin)
        self.threshold_plot.append(threshold25)

        # 5. State Machine for Upward Crossings
        if len(self.chosenBuf) > 20: 
            if self.prevChosenSmooth < self.prevThreshold and chosenSmooth >= threshold25:
                if self.pendingStrokeTime is None:
                    self.pendingStrokeTime = t_sec
                    self.pendingStrokeVal = chosenSmooth
                else:
                    if (t_sec - self.pendingStrokeTime) <= MIN_STROKE_TIME_SEC:
                        self.pendingStrokeTime = t_sec
                        self.pendingStrokeVal = chosenSmooth
                    else:
                        self._finalize_stroke(self.pendingStrokeTime)
                        self.crossings_t.append(self.pendingStrokeTime)
                        self.crossings_val.append(self.pendingStrokeVal)
                        
                        self.pendingStrokeTime = t_sec
                        self.pendingStrokeVal = chosenSmooth
                        
            elif self.pendingStrokeTime is not None and (t_sec - self.pendingStrokeTime) > MIN_STROKE_TIME_SEC:
                self._finalize_stroke(self.pendingStrokeTime)
                self.crossings_t.append(self.pendingStrokeTime)
                self.crossings_val.append(self.pendingStrokeVal)
                self.pendingStrokeTime = None

        self.prevChosenSmooth = chosenSmooth
        self.prevThreshold = threshold25


def stream_csv_data(filename):
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
fig.canvas.manager.set_window_title('C++ Logic Simulator (EMA Version)')

# Plot 1
line_x, = axs[0].plot([], [], label='EMA X', alpha=0.8, color='#1f77b4')
line_y, = axs[0].plot([], [], label='EMA Y', alpha=0.8, color='#ff7f0e')
line_z, = axs[0].plot([], [], label='EMA Z', alpha=0.8, color='#2ca02c')
axs[0].set_title('1. EMA Smoothed Acceleration (Alpha=0.3)')
axs[0].set_ylabel('Accel (g)')
axs[0].legend(loc='upper right')
axs[0].grid(True); axs[0].set_ylim(-3, 3)

# Plot 2
line_dom, = axs[1].plot([], [], label='Selected Axis (Variance)', color='purple', linewidth=2)
axs[1].set_title('2. Dynamically Chosen Axis (1=X, 2=Y, 3=Z)')
axs[1].set_ylabel('Axis Index')
axs[1].set_yticks([1, 2, 3]); axs[1].set_ylim(0.5, 3.5)
axs[1].grid(True, axis='y')

# Plot 3
line_chosen, = axs[2].plot([], [], label='Double EMA Axis', color='black', linewidth=1.5)
line_thresh, = axs[2].plot([], [], label='Lower 25% Threshold', color='blue', linestyle='--', alpha=0.8)
scat_strokes = axs[2].scatter([], [], color='green', s=50, zorder=5, label='Detected Stroke')
axs[2].set_title('3. Stroke Detection')
axs[2].set_ylabel('Accel (g)')
axs[2].legend(loc='upper right')
axs[2].grid(True); axs[2].set_ylim(-3, 3)

# Plot 4
line_raw_spm, = axs[3].step([], [], where='post', color='gray', alpha=0.3, label='Raw SPM')
line_spm, = axs[3].plot([], [], label='Filtered SPM', color='red', linewidth=2.5)
axs[3].set_title('4. Calculated Stroke Rate')
axs[3].set_xlabel('Time (seconds)')
axs[3].set_ylabel('SPM')
axs[3].set_ylim(10, 60)
axs[3].legend(loc='upper right')
axs[3].grid(True)

def update(frame):
    CHUNK_SIZE = 9  # 90ms of data per frame
    latest_t = 0
    for _ in range(CHUNK_SIZE):
        try:
            t, x, y, z = next(data_generator)
            processor.process_new_data(t, x, y, z, 5.0) 
            latest_t = t
        except StopIteration:
            return line_x, line_y, line_z, line_dom, line_chosen, line_thresh, scat_strokes, line_raw_spm, line_spm

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
        
    line_raw_spm.set_data(processor.raw_spm_t, processor.raw_spm_val)
    line_spm.set_data(processor.spm_t, processor.spm_val)

    current_t = processor.t_plot[-1]
    min_x = max(0, current_t - PLOT_WINDOW_SEC)
    max_x = max(PLOT_WINDOW_SEC, current_t)
    for ax in axs: ax.set_xlim(min_x, max_x)

    return line_x, line_y, line_z, line_dom, line_chosen, line_thresh, scat_strokes, line_raw_spm, line_spm

ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()