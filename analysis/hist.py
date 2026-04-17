import csv
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
CSV_FILE = 'data/athome.csv'

def plot_sensor_intervals(filename):
    imu_timestamps = []
    gps_timestamps = []

    print(f"Loading data from {filename}...")
    
    # 1. Read all rows and grab the timestamps
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or len(row) < 2: 
                    continue
                
                sensor_type = row[0]
                try:
                    ts = int(row[1])
                    if sensor_type == 'IMU':
                        imu_timestamps.append(ts)
                    elif sensor_type == 'GPS':
                        gps_timestamps.append(ts)
                except ValueError:
                    continue
    except FileNotFoundError:
        print("File not found.")
        return

    print(f"Found {len(imu_timestamps)} IMU packets and {len(gps_timestamps)} GPS packets.")

    if not imu_timestamps and not gps_timestamps:
        print("No valid timestamps found.")
        return

    # 2. Calculate the time difference (dt) between consecutive packets
    imu_dt = np.diff(imu_timestamps) if len(imu_timestamps) > 1 else []
    gps_dt = np.diff(gps_timestamps) if len(gps_timestamps) > 1 else []

    # Print some quick text stats
    if len(imu_dt) > 0:
        print(f"\nIMU Stats -> Mean Interval: {np.mean(imu_dt):.1f} ms | Min: {np.min(imu_dt)} ms | Max: {np.max(imu_dt)} ms")
    if len(gps_dt) > 0:
        print(f"GPS Stats -> Mean Interval: {np.mean(gps_dt):.1f} ms | Min: {np.min(gps_dt)} ms | Max: {np.max(gps_dt)} ms")

    # 3. Plotting
    plt.figure(figsize=(12, 10))

    # --- PLOT 1: IMU Histogram ---
    plt.subplot(2, 1, 1)
    if len(imu_dt) > 0:
        # Calculate 99th percentile to zoom the histogram past massive dropouts (like 3-second gaps)
        imu_max_plot = np.percentile(imu_dt, 99.5) if len(imu_dt) > 10 else np.max(imu_dt)
        
        # Plot up to the 99.5th percentile to keep the graph readable
        bins = np.linspace(0, imu_max_plot, 100)
        plt.hist(imu_dt, bins=bins, color='blue', edgecolor='black', alpha=0.7)
        plt.axvline(np.mean(imu_dt), color='red', linestyle='dashed', linewidth=2, label=f'Mean: {np.mean(imu_dt):.1f} ms')
        
    plt.title('IMU Sensor: Time Between Packets (Interval Distribution)')
    plt.xlabel('Time Difference between consecutive rows (milliseconds)')
    plt.ylabel('Number of Packets')
    plt.yscale('log')
    plt.grid(axis='y', alpha=0.7)
    plt.legend()

    # --- PLOT 2: GPS Histogram ---
    plt.subplot(2, 1, 2)
    if len(gps_dt) > 0:
        gps_max_plot = np.percentile(gps_dt, 99.5) if len(gps_dt) > 10 else np.max(gps_dt)
        bins = np.linspace(0, gps_max_plot, 100)
        
        plt.hist(gps_dt, bins=bins, color='green', edgecolor='black', alpha=0.7)
        plt.axvline(np.mean(gps_dt), color='red', linestyle='dashed', linewidth=2, label=f'Mean: {np.mean(gps_dt):.1f} ms')
        
    plt.title('GPS Sensor: Time Between Packets (Interval Distribution)')
    plt.xlabel('Time Difference between consecutive rows (milliseconds)')
    plt.ylabel('Number of Packets')
    plt.yscale('log')
    plt.grid(axis='y', alpha=0.7)
    plt.legend()

    plt.tight_layout()
    plt.savefig('graph_hardware_intervals.png')
    print("\n✅ Saved graph as 'graph_hardware_intervals.png'")
    plt.show()

if __name__ == "__main__":
    plot_sensor_intervals(CSV_FILE)