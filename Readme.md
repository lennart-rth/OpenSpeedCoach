# Open SpeedCoach

A GPS and IMU based speedometer for rowing.\
Less accurate than the NK Speedcoach, but 15x cheaper.

![](/banner.png)

## Features

- pace, distance, strokerate, time
- 2.7" e-paper display for goad readability outdoors
- Session logging to a micro SD card
- Transfer files to phone using bluetooth
- Acceleration curve analysis possible as raw data gets logged

## Hardware

- NiceNano nRF52 supermini
- generic cheap GPS module
- MPU6050 IMU
- 2.7" e-paper display
- Micro SD card
- USB-C (for charging only)
- 2000mah battery
- waterproof switch
- waterproof usb-c port

Pin assignment can be found in `include/DataTypes.h`.

## Getting Started

1. Add the board support package by following the instructions at [Nicenano-NRF52-Supermini-PlatformIO-Support](https://github.com/ICantMakeThings/Nicenano-NRF52-Supermini-PlatformIO-Support).
2. Open the project in PlatformIO.
3. Build and upload to the board:

   ```
   pio run -t upload
   ```

4. Open the serial monitor to follow startup and health messages:

   ```
   pio device monitor
   ```

On boot the device waits for a solid GPS fix (at least 5 satellites and an HDOP below 2.0) before it starts the session. Keep the device outside with a clear view of the sky while it searches.

## How It Works

- The GPS is polled continuously at roughly 1 Hz.
- The IMU is sampled at 100 Hz in a background RTOS task.
- A dead reckoning filter ignores GPS jumps under 2 meters so a stationary boat does not drift the total distance.
- Stroke rate is detectde based on the acceleration profile and dynamic updating thresholds on a rolling window.
- The display updates every 3 seconds, logging happens every 10 seconds.

## Data Logging

Each session creates a CSV file named after the GPS time it started, for example `08271412.csv`. The file starts with a header row, followed by rows tagged by type:

| Row type | Data |
| --- | --- |
| GPS | Timestamp, latitude, longitude, speed (km/h), distance from start (m), satellite count, HDOP, altitude, course |
| IMU | Timestamp, acceleration x/y/z, gyro x/y/z |
| SPM | Timestamp, stroke rate |

All times are device uptime in milliseconds.

Every boot also writes an event log to the SD card (`MMDDHHMM_events.log`) with health messages, warnings and errors, which is useful for diagnosing issues.

### Charging

Use a charge-only USB-C cable (no data lines). keep the power button on (connected). Fully charged when one of the LEDs on the nice nano turns of.

## Configuration

Tunable values live in `include/DataTypes.h`:

| Constant | Default | Purpose |
| --- | --- | --- |
| `LOG_INTERVAL_MS` | 10000 | How often buffered data is flushed to the SD card |
| `DISPLAY_INTERVAL_MS` | 3000 | How often the display refreshes |
| `IMU_INTERVAL_MS` | 10 | IMU sampling period, 100 Hz |
| `DEADRECONING_DISTANCE_THRESHOLD` | 2.0 | Minimum GPS step in meters counted as movement |

