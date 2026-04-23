import pandas as pd
import plotly.graph_objects as go

def plot_gps_data(file_path):
    # 1. Parse the custom CSV format to extract only GPS data
    gps_records = []
    
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            
            # Only process lines that start with 'GPS'
            if parts[0] == 'GPS':
                if len(parts) < 10 or not parts[1].isdigit():
                    continue
                
                gps_records.append({
                    'timestamp_ms': int(parts[1]),
                    'lat': float(parts[2]),
                    'lon': float(parts[3]),
                    'speed': float(parts[4]),
                    'satellites': int(parts[6]),
                    'hdop': float(parts[7]),
                    'altitude': float(parts[8]),
                    'course': float(parts[9])
                })

    # Convert to a pandas DataFrame for easy manipulation
    df = pd.DataFrame(gps_records)
    
    if df.empty:
        print("No GPS data found in the file.")
        return

    # 2. Calculate dynamic point size
    # We want points smaller when there are MORE satellites.
    # Inverse relationship: size = constant / satellites. 
    # We use .clip(lower=1) to prevent division-by-zero errors if satellites drop to 0.
    base_marker_size = 25
    df['point_size'] = base_marker_size / df['satellites'].clip(lower=1)

    # 3. Build the interactive map
    fig = go.Figure()

    # Add the continuous track (line connecting the points)
    fig.add_trace(go.Scattermapbox(
        mode="lines",
        lon=df['lon'],
        lat=df['lat'],
        line=dict(width=3, color='gray'),
        name="GPS Track"
    ))

    # Add the individual GPS points (sized by satellites, colored by speed)
    fig.add_trace(go.Scattermapbox(
        mode="markers",
        lon=df['lon'],
        lat=df['lat'],
        marker=dict(
            size=df['point_size'],
            color=df['speed'],
            colorscale='Turbo', # 'Turbo' is a great color scale for highlighting intensity (speed)
            showscale=True,
            colorbar=dict(title="Speed")
        ),
        # What to show when hovering your mouse over a point
        text=df.apply(lambda row: f"Speed: {row['speed']} <br>Satellites: {int(row['satellites'])}", axis=1),
        hoverinfo="text",
        name="Data Points"
    ))

    # 4. Configure map layout and set the starting view
    fig.update_layout(
        mapbox=dict(
            style="open-street-map",
            center=dict(lat=df['lat'].mean(), lon=df['lon'].mean()),
            zoom=17 # Adjust this depending on how tight your tracking cluster is
        ),
        margin=dict(l=0, r=0, t=0, b=0), # Removes white borders
        title="GPS Track Analysis"
    )

    # 5. Display the map in your default web browser
    fig.show()

if __name__ == "__main__":
    # Replace 'data.csv' with the actual path to your file
    plot_gps_data('data/0423_1649.csv')