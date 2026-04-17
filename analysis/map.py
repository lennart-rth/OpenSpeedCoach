import csv
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
CSV_FILE = 'data/athome.csv'

def plot_gps_session(filename):
    ts_ms = []
    lats = []
    lons = []
    speeds = []
    sats = []

    print(f"Reading GPS data from {filename}...")
    
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0] != 'GPS': continue
                try:
                    # GPS,Millis,Lat,Lon,Speed,DistToStart,Sats
                    ts_ms.append(int(row[1]))
                    lats.append(float(row[2]))
                    lons.append(float(row[3]))
                    speeds.append(float(row[4]))
                    sats.append(int(row[6]))
                except (ValueError, IndexError):
                    continue
    except FileNotFoundError:
        print("File not found.")
        return

    if not lats:
        print("No GPS data found.")
        return

    # Convert to numpy arrays
    t_sec = (np.array(ts_ms) - ts_ms[0]) / 1000.0
    lats = np.array(lats)
    lons = np.array(lons)
    speeds = np.array(speeds)

    # --- PLOTTING ---
    fig = plt.figure(figsize=(12, 10))

    # 1. THE "MAP" (Lat/Lon Scatter)
    ax1 = plt.subplot(2, 1, 1)
    # Plot the track line
    ax1.plot(lons, lats, color='blue', alpha=0.5, label='Path')
    # Use scatter to show speed intensity on the map
    sc = ax1.scatter(lons, lats, c=speeds, cmap='jet', s=10, label='Speed Points')
    
    # Mark Start and End
    ax1.plot(lons[0], lats[0], 'go', markersize=10, label='START')
    ax1.plot(lons[-1], lats[-1], 'ro', markersize=10, label='END')
    
    plt.colorbar(sc, ax=ax1, label='Speed (km/h)')
    ax1.set_title('GPS Track (Coordinate Map)')
    ax1.set_xlabel('Longitude')
    ax1.set_ylabel('Latitude')
    ax1.grid(True)
    ax1.legend()
    # Ensure the aspect ratio is equal so the map isn't stretched
    ax1.set_aspect('equal', 'datalim')

    # 2. SPEED OVER TIME
    ax2 = plt.subplot(2, 1, 2)
    ax2.fill_between(t_sec, speeds, color='green', alpha=0.3)
    ax2.plot(t_sec, speeds, color='green', linewidth=1.5)
    
    # Calculate average speed (excluding zeros if stationary)
    avg_speed = np.mean(speeds[speeds > 1.0]) if any(speeds > 1.0) else 0
    ax2.axhline(avg_speed, color='red', linestyle='--', label=f'Avg Active: {avg_speed:.2f} km/h')
    
    ax2.set_title('Speed Profile Over Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Speed (km/h)')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.savefig('session_map_report.png')
    print("\n✅ Report saved as 'session_map_report.png'")
    plt.show()

if __name__ == "__main__":
    plot_gps_session(CSV_FILE)