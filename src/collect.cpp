#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// --- CONFIGURATION ---
const int LOG_INTERVAL_MS = 20000; // Update Display/SD every 20s
const int IMU_INTERVAL_MS = 25;    // 40Hz = 1000ms / 25ms

// Max buffers for 20 seconds of data
const int MAX_IMU_SAMPLES = 850;   // 20s * 40Hz = 800 (added margin)
const int MAX_GPS_SAMPLES = 25;    // 20s * 1Hz = 20 (added margin)

// --- PINS (Kept from your code) ---
#define SD_CS    PIN_020
#define EPD_CS   PIN_024
#define EPD_DC   PIN_022
#define EPD_RST  PIN_104
#define EPD_BUSY PIN_106
#define PIN_MISO PIN_017
#define PIN_MOSI PIN_011
#define PIN_SCK  PIN_100
#define MPU_SDA  PIN_029
#define MPU_SCL  PIN_031
#define GPS_RX_PIN PIN_115
#define GPS_TX_PIN PIN_002

// --- DATA STRUCTURES ---
struct ImuData {
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct GpsData {
  uint32_t timestamp;
  double lat;
  double lon;
  float speed;
  float distToStart;
  uint8_t sats;
};

// --- BUFFERS ---
ImuData imuBuffer[MAX_IMU_SAMPLES];
GpsData gpsBuffer[MAX_GPS_SAMPLES];
int imuCount = 0;
int gpsCount = 0;

// --- OBJECTS ---
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));
SdFat sd;
File32 myFile;
Adafruit_MPU6050 mpu;
// Initializing Wire explicitly for nRF52 if needed, otherwise standard Wire works
TwoWire MyWire = TwoWire(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, MPU_SDA, MPU_SCL);
TinyGPSPlus gps;

// --- GLOBAL STATE ---
bool sdOK = false;
bool mpuOK = false;
bool fixFound = false;
char logFileName[16] = "temp.csv";
String errorMessage = ""; // Holds runtime errors

double startLat = 0, startLon = 0;
double lastLat = 0, lastLon = 0;
double totalDist = 0.0;
unsigned long startTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastImuTime = 0;

// --- HELPER FUNCTIONS ---

void logError(String msg) {
  errorMessage = msg;
  Serial.println(msg);
}

// Draw the Setup Screen
void drawSetupScreen(String timeStr) {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    // Title
    display.setCursor(5, 30);
    display.setTextSize(2);
    display.print("OpenSpeedCoach");

    // Time (Medium Large)
    display.setCursor(5, 60);
    display.setTextSize(3);
    display.print(timeStr);

    // Component Status (Small)
    display.setTextSize(1);
    display.setCursor(5, 90);
    display.print("SD Card: "); 
    if(sdOK) display.print("OK"); else display.print("ERROR");

    display.setCursor(5, 105);
    display.print("Accelerometer: ");
    if(mpuOK) display.print("OK"); else display.print("ERROR");

    // Bottom Status
    display.setCursor(5, 140);
    display.setTextSize(2);
    display.print("Find Satellite");
    display.setCursor(5, 160);
    display.print("Lock...");

  } while (display.nextPage());
}

// Draw Main Loop Screen
void drawMainScreen(float avgSpeed, float dist, float minutes, int sats) {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    // 1. Top Bar: Error Message (if any) or Sats
    display.setCursor(2, 10);
    display.setTextSize(1);
    if(errorMessage != "") {
      display.print("ERR: "); display.print(errorMessage);
    } else {
      display.print("Sats: "); display.print(sats);
      if (sdOK) display.print(" SD:Rec");
    }

    // 2. Main Speed (Avg last 10s)
    display.drawLine(0, 15, display.width(), 15, GxEPD_BLACK);
    
    display.setCursor(5, 30);
    display.setTextSize(2);
    display.print("AVG SPEED (10s)");

    display.setCursor(10, 80);
    display.setTextSize(5); // Big text
    display.print(avgSpeed, 1);
    display.setTextSize(2);
    display.print(" km/h");

    // 3. Bottom Stats: Distance and Time
    display.drawLine(0, 100, display.width(), 100, GxEPD_BLACK);
    
    // Dist
    display.setCursor(5, 120);
    display.setTextSize(2);
    display.print("Dist: ");
    display.print((int)dist);
    display.print(" m");

    // Time
    display.setCursor(5, 150);
    display.print("Time: ");
    display.print(minutes, 1);
    display.print(" m");

  } while (display.nextPage());
}

void flushDataToSD() {
  if (!sdOK) return;

  // Open file in append mode
  if (myFile.open(logFileName, O_RDWR | O_CREAT | O_APPEND)) {
    
    // Write GPS Chunk
    for (int i = 0; i < gpsCount; i++) {
      myFile.print("GPS,");
      myFile.print(gpsBuffer[i].timestamp); myFile.print(",");
      myFile.print(gpsBuffer[i].lat, 6); myFile.print(",");
      myFile.print(gpsBuffer[i].lon, 6); myFile.print(",");
      myFile.print(gpsBuffer[i].speed); myFile.print(",");
      myFile.print(gpsBuffer[i].distToStart); myFile.print(",");
      myFile.println(gpsBuffer[i].sats);
    }

    // Write IMU Chunk
    for (int i = 0; i < imuCount; i++) {
      myFile.print("IMU,");
      myFile.print(imuBuffer[i].timestamp); myFile.print(",");
      myFile.print(imuBuffer[i].ax); myFile.print(",");
      myFile.print(imuBuffer[i].ay); myFile.print(",");
      myFile.print(imuBuffer[i].az); myFile.print(",");
      myFile.print(imuBuffer[i].gx); myFile.print(",");
      myFile.print(imuBuffer[i].gy); myFile.print(",");
      myFile.println(imuBuffer[i].gz);
    }
    
    myFile.close();
  } else {
    logError("SD Write Fail");
    sdOK = false; // Stop trying if it failed
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  // 1. Init Hardware
  MyWire.begin();
  mpuOK = mpu.begin(MPU6050_I2CADDR_DEFAULT, &MyWire);
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  Serial1.setPins(GPS_RX_PIN, GPS_TX_PIN);
  Serial1.begin(9600);

  SPI.setPins(PIN_MISO, PIN_SCK, PIN_MOSI);
  sdOK = sd.begin(SD_CS, SD_SCK_MHZ(12));

  display.init(115200);
  display.setRotation(3); // Adjust as needed (0-3)

  // 2. Initial Setup Screen (Time unknown yet)
  drawSetupScreen("--:--");

  // 3. Wait for Lock
  // We loop here until we have a valid location. 
  // We can update the time on screen if we get time packets before location lock.
  
  unsigned long lastScreenUpdate = 0;
  
  while (!fixFound) {
    while (Serial1.available() > 0) gps.encode(Serial1.read());

    if (gps.location.isValid() && gps.date.year() > 2000) {
      fixFound = true;
      startLat = gps.location.lat();
      startLon = gps.location.lng();
      lastLat = startLat;
      lastLon = startLon;
      startTime = millis();
      lastLogTime = millis();
      lastImuTime = millis();

      // Generate Filename
      if (sdOK) {
        sprintf(logFileName, "%02d%02d%02d%02d.csv",
                gps.date.month(), gps.date.day(),
                gps.time.hour(), gps.time.minute());
        
        if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
          myFile.println("Type,Millis,Data1,Data2,Data3,Data4,Data5,Data6");
          myFile.close();
        } else {
          sdOK = false;
        }
      }
    } else {
        // Optional: Update time on screen if changed while waiting for lock
        if (gps.time.isUpdated() && millis() - lastScreenUpdate > 60000) {
           char timeBuf[10];
           sprintf(timeBuf, "%02d:%02d", gps.time.hour(), gps.time.minute());
           drawSetupScreen(String(timeBuf));
           lastScreenUpdate = millis();
        }
    }
  }
}

// --- MAIN LOOP ---
void loop() {
  unsigned long currentMillis = millis();

  // 1. Process GPS Raw Data (Continuous)
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      // If we have a new location fix
      if (gps.location.isValid()) {
        // Calculate Distance Accumulation
        double distStep = gps.distanceBetween(gps.location.lat(), gps.location.lng(), lastLat, lastLon);
        // Noise filter: only add if moved > 1.0 meter
        if (distStep > 1.0) {
          totalDist += distStep;
          lastLat = gps.location.lat();
          lastLon = gps.location.lng();
        }

        // Add to Buffer (Low Speed 1Hz)
        if (gpsCount < MAX_GPS_SAMPLES) {
          gpsBuffer[gpsCount].timestamp = millis();
          gpsBuffer[gpsCount].lat = gps.location.lat();
          gpsBuffer[gpsCount].lon = gps.location.lng();
          gpsBuffer[gpsCount].speed = gps.speed.kmph();
          gpsBuffer[gpsCount].distToStart = gps.distanceBetween(gps.location.lat(), gps.location.lng(), startLat, startLon);
          gpsBuffer[gpsCount].sats = gps.satellites.value();
          gpsCount++;
        }
      }
    }
  }

  // 2. Process IMU Data (40Hz Timer)
  if (currentMillis - lastImuTime >= IMU_INTERVAL_MS) {
    lastImuTime = currentMillis;
    
    if (mpuOK && imuCount < MAX_IMU_SAMPLES) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Store raw values to save processing time/storage space
      // You can convert to float later during data analysis
      // MPU raw values are usually accessible, but Adafruit library gives floats.
      // We will cast floats to int16 for compactness if we accessed registers directly,
      // but here we just cast the float * 100 or keep it simple. 
      // Let's store the float values in the struct but simplified. 
      // Actually, Adafruit MPU getEvent returns floats (m/s^2). 
      // To save RAM/SD Speed, let's just store the integer part * 100 or similar? 
      // For now, let's keep the struct logic simple:
      
      imuBuffer[imuCount].timestamp = currentMillis;
      imuBuffer[imuCount].ax = (int16_t)(a.acceleration.x * 100);
      imuBuffer[imuCount].ay = (int16_t)(a.acceleration.y * 100);
      imuBuffer[imuCount].az = (int16_t)(a.acceleration.z * 100);
      imuBuffer[imuCount].gx = (int16_t)(g.gyro.x * 100);
      imuBuffer[imuCount].gy = (int16_t)(g.gyro.y * 100);
      imuBuffer[imuCount].gz = (int16_t)(g.gyro.z * 100);
      imuCount++;
    }
  }

  // 3. Process Display & Log (20s Timer)
  if (currentMillis - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentMillis;

    // A. Calculate Averages
    // We want the average speed of the LAST 10 SECONDS.
    // Since gpsCount contains ~20 seconds of data, we look at the second half of the array.
    float speedSum = 0;
    int speedSamples = 0;
    
    // We iterate backwards from the current end of buffer
    unsigned long cutoffTime = currentMillis - 10000; 

    for (int i = 0; i < gpsCount; i++) {
      if (gpsBuffer[i].timestamp >= cutoffTime) {
        speedSum += gpsBuffer[i].speed;
        speedSamples++;
      }
    }
    
    float avgSpeed10s = (speedSamples > 0) ? (speedSum / speedSamples) : 0.0;
    float totalTimeMin = (currentMillis - startTime) / 60000.0;

    // B. Update Display
    // Note: E-paper update blocks for ~2 seconds. No IMU data collected during this time.
    drawMainScreen(avgSpeed10s, totalDist, totalTimeMin, gps.satellites.value());

    // C. Flush to SD
    flushDataToSD();

    // D. Reset Buffers
    imuCount = 0;
    gpsCount = 0;
  }
}
