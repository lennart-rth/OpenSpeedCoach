import csv
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
CSV_FILE = 'data/04161551.csv'

def run_sensitivity_analysis(filename):
    ts_ms, lats, lons, speeds = [], [], [], []
    
    # 1. Load Data
    print(f"Loading {filename}...")
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0] != 'GPS': continue
            try:
                ts_ms.append(int(row[1]))
                lats.append(float(row[2]))
                lons.append(float(row[3]))
                speeds.append(float(row[4])) # Speed in km/h
            except (ValueError, IndexError): continue
    
    lats = np.array(lats)
    lons = np.array(lons)
    speeds = np.array(speeds)
    
    def get_dist(lat1, lon1, lat2, lon2):
        # Haversine approximation for small distances
        dy = (lat2 - lat1) * 111111
        dx = (lon2 - lon1) * 111111 * np.cos(np.radians(lat1))
        return np.sqrt(dx**2 + dy**2)

    # 2. Sweep Analysis
    # We sweep Distance Threshold while keeping a fixed Speed Threshold of 1.5 km/h
    thresholds = np.linspace(0, 5, 50) 
    speed_gate = 1.5 # km/h (Ignore jitter if speed is below this)
    
    total_distances = []
    stationary_noise = [] # Distance accumulated while GPS says speed < 0.5 km/h

    is_stationary = speeds < 0.5 

    for thresh in thresholds:
        dist_total = 0
        dist_noise = 0
        last_lat, last_lon = lats[0], lons[0]
        
        for i in range(1, len(lats)):
            d = get_dist(last_lat, last_lon, lats[i], lons[i])
            
            # HYBRID LOGIC:
            # Move is valid if (Dist > Thresh) OR (Speed > Gate)
            if d > thresh or speeds[i] > speed_gate:
                dist_total += d
                if is_stationary[i]:
                    dist_noise += d
                last_lat, last_lon = lats[i], lons[i]
        
        total_distances.append(dist_total)
        stationary_noise.append(dist_noise)

    total_distances = np.array(total_distances)
    stationary_noise = np.array(stationary_noise)

    # 3. Robust Knee Detection
    # Calculate second derivative to find where the curve flattens
    dy = np.gradient(total_distances)
    d2y = np.gradient(dy)
    elbow_idx = np.argmax(d2y)
    recommended_thresh = thresholds[elbow_idx]

    # --- PLOTTING ---
    plt.figure(figsize=(12, 10))

    # Plot 1: Total Distance vs Threshold
    plt.subplot(2, 1, 1)
    plt.plot(thresholds, total_distances / 1000.0, 'b-o', markersize=4, label='Hybrid Distance (km)')
    plt.axvline(recommended_thresh, color='red', linestyle='--', label=f'Recommended: {recommended_thresh:.2f}m')
    plt.ylabel('Total Distance (km)')
    plt.title('Sensitivity Analysis: Hybrid Dead Reckoning (Dist + Speed)')
    plt.grid(True)
    plt.legend()

    # Plot 2: Signal Loss vs Stationary Accumulation
    plt.subplot(2, 1, 2)
    plt.plot(thresholds, stationary_noise, 'r-', label='Stationary Wobble (m)')
    plt.ylabel('Noise Accumulated (meters)')
    plt.xlabel('Distance Threshold (meters)')
    
    # Secondary axis for the "Knee" visualization
    ax2 = plt.twinx()
    ax2.plot(thresholds, d2y, 'g--', alpha=0.5, label='Curvature (Knee)')
    ax2.set_ylabel('Curvature Intensity')
    
    plt.title('Optimization: Eliminating Stationary Jitter')
    plt.grid(True)
    plt.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig('dead_rec_hybrid_analysis.png')
    print(f"--- Analysis Complete ---")
    print(f"Recommended Distance Threshold: {recommended_thresh:.2f} meters")
    print(f"Applied Speed Gate: {speed_gate} km/h")
    plt.show()

if __name__ == "__main__":
    run_sensitivity_analysis(CSV_FILE)