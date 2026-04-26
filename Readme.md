# Open SpeedCoach

Follow https://github.com/ICantMakeThings/Nicenano-NRF52-Supermini-PlatformIO-Support to add board to platformio




# Charging
- need a non data usb-c cable
- is full when one of the led turns off. I forgott if it was the blue or red one.


# main.cpp

## 💾 What is being written to the SD Card?
File Naming: It creates a new CSV file every time the device gets a GPS lock. The filename is generated using the current GPS date and time (Format: MMDDHHMM.csv).

Batch Writing: It does not write to the SD card every single loop. It stores 20 readings in a buffer (BUFFER_SIZE = 20) and flushes them to the SD card all at once. This prevents the SD card from bottlenecking the system.

The Header: The first row written to the file is: Millis,Lat,Lon,Speed,DistStart,Sats.

The Logged Data (per row):

time: Device uptime in milliseconds (millis()).

lat: Latitude (saved to 6 decimal places).

lon: Longitude (saved to 6 decimal places).

speed: Current speed in km/h.

distToStart: The straight-line distance (in meters) from where the device first got its GPS lock.

sats: The number of GPS satellites currently connected.

## ⚙️ How does the Calibration / Initialization work?
Based on the provided code, here is how the sensors are initialized and filtered:

IMU (MPU6050) Calibration: * There is no active mathematical calibration (like calculating offsets or zeroing out drift at rest) happening in this code.

It only performs hardware configuration, setting the accelerometer to a range of ±8G, the gyroscope to ±500 degrees/second, and applying a 21Hz hardware low-pass filter to smooth out raw sensor noise. (Note: The code initializes the MPU, but doesn't actually read or use its data in the main loop yet—the Stroke Rate quadrant just prints "0.0").

## GPS "Calibration" (Initial Fix & Distance Filtering):

Startup Lock: The code traps the device on a setup screen until a valid GPS fix is found. It takes the very first valid coordinate and sets it as the "Start Location" (startLat, startLon).

Anti-Drift Filter: GPS signals naturally "jitter," which can make a stationary device look like it is moving back and forth, falsely adding to your total distance. To counter this, the code requires a minimum movement step: it will only add to your totalDist if the distance between the last coordinate and the new coordinate is greater than 2.0 meters (distStep > 2.0).

Speed Smoothing: To prevent the on-screen speed from jumping around erratically, the display doesn't show instantaneous speed. It calculates a rolling average of the last 10 speed readings (AVG_WINDOW = 10) before updating the screen.


## Data writing rate

GPS Data Rate: ~1 Hz (Saved to SD every ~20 seconds)

Sampling: The code grabs a new GPS coordinate every time the GPS module outputs a valid update. For standard GPS modules running at 9600 baud, this is usually 1 time per second (1 Hz).

Writing: You are batching the data. The code waits until the buffer fills up with 20 readings (BUFFER_SIZE = 20). This means an actual physical write to the SD card only happens roughly once every 20 seconds.

Accelerometer Data Rate: 0 Hz (It is not being written at all)

The Catch: While your code successfully connects to the MPU6050 in the setup() function and shows an icon on the screen, you are never reading or saving its data.

Your RowData structure does not contain variables for pitch, roll, or acceleration, and the flushBufferToSD() function only writes the GPS variables to the file.





# TODO
- show coahing info like powercurve, acceleartion data or how much the boat is moving below the body on the recovery