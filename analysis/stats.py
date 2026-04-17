import csv
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
CSV_FILE = 'data/04161551.csv'  # REPLACE with your filename

def run_diagnostics(filename):
    timestamps_ms = []
    accel_mags = []

    print(f"--- Loading Data: {filename} ---")
    
    # 1. LOAD DATA
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
                    
                    mag = np.sqrt(ax**2 + ay**2 + az**2)
                    timestamps_ms.append(ts)
                    accel_mags.append(mag)
                except (ValueError, IndexError): 
                    continue
    except FileNotFoundError:
        print("Error: File not found.")
        return

    if not timestamps_ms:
        print("No valid IMU data found in the file.")
        return

    # Convert to NumPy arrays for vectorized math
    ts = np.array(timestamps_ms)
    mags = np.array(accel_mags)
    
    # Time in seconds from start
    t_sec = (ts - ts[0]) / 1000.0

    # ==========================================
    # PART 1: Time & Quality Analysis
    # ==========================================
    print("\n--- 1. Data Quality & Timing ---")
    
    # Calculate time differences (dt) between consecutive samples
    dt_ms = np.diff(ts)
    
    mean_dt = np.mean(dt_ms)
    std_dt = np.std(dt_ms)
    max_dt = np.max(dt_ms)
    min_dt = np.min(dt_ms)
    
    # Actual sampling frequency based on average dt
    actual_fs = 1000.0 / mean_dt if mean_dt > 0 else 0

    print(f"Total Samples:      {len(ts)}")
    print(f"Total Duration:     {t_sec[-1]:.2f} seconds")
    print(f"Average Sampling:   {actual_fs:.2f} Hz")
    print(f"Average Interval:   {mean_dt:.2f} ms")
    print(f"Interval Jitter:    ±{std_dt:.2f} ms (Standard Deviation)")
    print(f"Max Gap (Dropouts): {max_dt:.2f} ms")
    print(f"Min Interval:       {min_dt:.2f} ms")

    if std_dt > 5.0:
        print("⚠️ WARNING: High jitter detected. Sensor timing is inconsistent.")
    if max_dt > (mean_dt * 3):
        print("⚠️ WARNING: Significant data dropouts detected (missing packets).")

    # ==========================================
    # PART 2: Statistical Analysis
    # ==========================================
    print("\n--- 2. Signal Statistics ---")
    
    mean_mag = np.mean(mags)
    std_mag = np.std(mags)
    var_mag = np.var(mags)
    
    print(f"Mean Magnitude:     {mean_mag:.4f} g (Should be ~9.81 if stationary)")
    print(f"Standard Deviation: {std_mag:.4f} g (Indicates overall movement intensity)")
    print(f"Signal Variance:    {var_mag:.4f}")
    print(f"Max Value:          {np.max(mags):.4f} g")
    print(f"Min Value:          {np.min(mags):.4f} g")

    # ==========================================
    # PART 3: FFT (Frequency Domain) Analysis
    # ==========================================
    print("\n--- 3. Frequency Analysis (FFT) ---")
    
    # Remove the DC component (gravity / mean) so it doesn't dominate the FFT
    mags_no_dc = mags - mean_mag
    
    # Number of samples
    N = len(mags_no_dc)
    
    # Perform Fast Fourier Transform
    fft_result = np.fft.fft(mags_no_dc)
    
    # Get the corresponding frequencies
    # We use the average interval in seconds for the FFT frequencies
    freqs = np.fft.fftfreq(N, d=(mean_dt / 1000.0))
    
    # We only care about positive frequencies (first half of the array)
    pos_mask = freqs > 0
    freqs_pos = freqs[pos_mask]
    
    # Calculate Magnitude Spectrum (normalize by N)
    fft_mag_pos = np.abs(fft_result[pos_mask]) * 2.0 / N
    
    # Find the dominant frequency
    max_freq_idx = np.argmax(fft_mag_pos)
    dominant_freq = freqs_pos[max_freq_idx]
    dominant_spm = dominant_freq * 60.0

    print(f"Dominant Frequency: {dominant_freq:.3f} Hz")
    print(f"Estimated Base SPM: {dominant_spm:.1f} Strokes/Minute")

    # ==========================================
    # PART 4: Plotting the Diagnostic Dashboard
    # ==========================================
    plt.figure(figsize=(12, 10))

    # Plot 1: The Raw Signal
    plt.subplot(3, 1, 1)
    plt.plot(t_sec, mags, label='Raw Magnitude', color='blue', alpha=0.7)
    plt.axhline(mean_mag, color='red', linestyle='--', label=f'Mean ({mean_mag:.2f})')
    plt.title('Time Domain: Acceleration Magnitude')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude (g)')
    plt.grid(True)
    plt.legend()

    # Plot 2: Sampling Interval Histogram (Checks Jitter)
    plt.subplot(3, 1, 2)
    plt.hist(dt_ms, bins=50, color='orange', edgecolor='black', alpha=0.7)
    plt.axvline(mean_dt, color='red', linestyle='dashed', linewidth=2, label=f'Mean dt ({mean_dt:.1f}ms)')
    plt.title('Data Quality: Sampling Interval Jitter')
    plt.xlabel('Time Between Samples (ms)')
    plt.ylabel('Count')
    plt.grid(axis='y')
    plt.legend()

    # Plot 3: Frequency Spectrum (FFT)
    plt.subplot(3, 1, 3)
    plt.plot(freqs_pos, fft_mag_pos, color='purple')
    # Focus the x-axis on human movement frequencies (0 to 10 Hz)
    plt.xlim(0, 10) 
    plt.axvline(dominant_freq, color='red', linestyle='--', label=f'Peak: {dominant_freq:.2f} Hz ({dominant_spm:.0f} SPM)')
    plt.title('Frequency Domain: FFT Spectrum')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig('diagnostic_dashboard.png')
    print("\n✅ Saved visual report as 'diagnostic_dashboard.png'")
    plt.show()

if __name__ == "__main__":
    run_diagnostics(CSV_FILE)