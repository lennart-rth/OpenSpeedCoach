import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# --- CONFIGURATION ---
CSV_FILE = 'data/0425_1937.csv'
FS = 50.0                           # Sampling rate (Hz)
DT = 1.0 / FS                       # Delta time per sample

# --- TUNING PARAMETERS ---
GRAVITY_CUTOFF = 0.5                # Hz (Isolates the slow-moving tilt of the boat)
FORWARD_AXIS = 1                    # Assuming Y-axis (index 1) is pointing to the bow.
ALPHA = 0.98                        # Complementary Filter weight (0.98 to 0.995 is standard)

def butter_lowpass_filter(data, cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def analyze_sensor_fusion(filename):
    print("--- 1. Loading Data ---")
    ts, ax, ay, az, gps_speed_kmh = [], [], [], [], []
    
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            first_t = None
            for row in reader:
                if not row: continue
                # Handle IMU rows
                if row[0] == 'IMU' and len(row) >= 8:
                    try:
                        t = int(row[1])
                        if first_t is None: first_t = t
                        # Convert to m/s^2
                        x = (float(row[2]) / 100.0) * 9.81
                        y = (float(row[3]) / 100.0) * 9.81
                        z = (float(row[4]) / 100.0) * 9.81
                        
                        ts.append((t - first_t) / 1000.0)
                        ax.append(x); ay.append(y); az.append(z)
                        
                        # Sample-and-hold the latest GPS reading
                        if len(gps_speed_kmh) > 0:
                            gps_speed_kmh.append(gps_speed_kmh[-1])
                        else:
                            gps_speed_kmh.append(0.0)
                            
                    except ValueError: continue
                
                # Handle GPS rows
                elif row[0] == 'GPS' and len(row) >= 5:
                    try:
                        speed = float(row[4])
                        if len(gps_speed_kmh) > 0:
                            gps_speed_kmh[-1] = speed # Update the latest padded value
                    except ValueError: continue
                    
    except FileNotFoundError:
        print(f"File {filename} not found.")
        return

    time_sec = np.array(ts)
    raw_accel = np.array([ax, ay, az])[FORWARD_AXIS] 
    gps_speed_kmh = np.array(gps_speed_kmh)
    
    # Convert GPS ground truth to m/s for the math
    gps_speed_ms = gps_speed_kmh / 3.6

    print("--- 2. Isolating Gravity ---")
    gravity_vector = butter_lowpass_filter(raw_accel, GRAVITY_CUTOFF, FS)
    
    print("--- 3. Extracting Linear Acceleration ---")
    linear_accel = raw_accel - gravity_vector

    print("--- 4. Complementary Filter (Sensor Fusion) ---")
    fused_velocity_ms = np.zeros_like(linear_accel)
    
    # Seed the first value with the starting GPS speed
    fused_velocity_ms[0] = gps_speed_ms[0]
    
    for i in range(1, len(linear_accel)):
        # FUSED_V = ALPHA * (Integration) + (1 - ALPHA) * (GPS_Baseline)
        fused_velocity_ms[i] = ALPHA * (fused_velocity_ms[i-1] + (linear_accel[i] * DT)) + (1.0 - ALPHA) * gps_speed_ms[i]

    # Convert the fused m/s back to km/h for the final display
    fused_speed_kmh = fused_velocity_ms * 3.6

    print("--- 5. Plotting Results ---")
    fig, axs = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

    # Plot 1: Raw vs Linear Accel
    axs[0].plot(time_sec, raw_accel, label='Raw Forward Accel', color='lightgray')
    axs[0].plot(time_sec, linear_accel, label='Linear Accel (Pure Push)', color='#1f77b4', alpha=0.9)
    axs[0].set_title('1. Gravity Compensation')
    axs[0].set_ylabel('Accel (m/s²)')
    axs[0].legend(loc='upper right')
    axs[0].grid(True)

    # Plot 2: GPS Baseline vs IMU Surge
    axs[1].plot(time_sec, gps_speed_kmh, label='Raw GPS Speed (Stair-stepped updates)', color='black', alpha=0.5, linestyle='--')
    axs[1].plot(time_sec, fused_speed_kmh, label=f'Fused Speed (Alpha = {ALPHA})', color='purple', linewidth=2)
    axs[1].set_title('2. Fused Real-Time Speed')
    axs[1].set_ylabel('Speed (km/h)')
    axs[1].legend(loc='upper right')
    axs[1].grid(True)

    # Plot 3: Zoomed in to see the intra-stroke profile clearly
    axs[2].plot(time_sec, fused_speed_kmh, label='Fused Instantaneous Pace', color='red', linewidth=2)
    axs[2].set_title('3. Final Speed Profile (Notice how it centers around the actual ~14km/h baseline!)')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel('Speed (km/h)')
    axs[2].legend(loc='upper right')
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_sensor_fusion(CSV_FILE)