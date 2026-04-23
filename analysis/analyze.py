import pandas as pd
import matplotlib.pyplot as plt

def plot_imu_data(file_path):
    # 1. Parse the CSV format to extract only IMU data
    imu_records = []
    
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            
            # Only process lines that start with 'IMU'
            if parts[0] == 'IMU':
                if len(parts) < 8:
                    continue
                
                try:
                    imu_records.append({
                        'timestamp_ms': int(parts[1]),
                        'ax': float(parts[2]),
                        'ay': float(parts[3]),
                        'az': float(parts[4]),
                        'gx': float(parts[5]),
                        'gy': float(parts[6]),
                        'gz': float(parts[7])
                    })
                except (ValueError, IndexError):
                    continue

    # 2. Convert to a pandas DataFrame
    df = pd.DataFrame(imu_records)
    
    if df.empty:
        print("No IMU data found in the file.")
        return

    # Sort the dataframe by the timestamp so the plot lines connect chronologically 
    df = df.sort_values(by='timestamp_ms')

    # 3. Plot the acceleration curves
    plt.figure(figsize=(12, 6))

    # Plotting ax, ay, az against the timestamp
    plt.plot(df['timestamp_ms'], df['ax'], label='ax (X-Axis)', linestyle='-', color='red', alpha=0.8)
    plt.plot(df['timestamp_ms'], df['ay'], label='ay (Y-Axis)', linestyle='-', color='green', alpha=0.8)
    plt.plot(df['timestamp_ms'], df['az'], label='az (Z-Axis)', linestyle='-', color='blue', alpha=0.8)

    # Styling the plot
    plt.title('IMU Acceleration Data over Time')
    plt.xlabel('Time (milliseconds)')
    plt.ylabel('Raw Acceleration')
    plt.legend(loc='upper right')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()

    # Display the plot in an interactive window
    plt.show()

if __name__ == "__main__":
    # Replace 'data.csv' with the actual path to your file
    plot_imu_data('data/0423_1604.csv')