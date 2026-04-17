#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

// --- CONFIGURATION ---
const int LOG_INTERVAL_MS = 20000;     // SD Card write interval
const int DISPLAY_INTERVAL_MS = 3000;  // Display & SPM calc interval
const int IMU_INTERVAL_MS = 10;    

const int MAX_IMU_SAMPLES = 1050;   
const int MAX_GPS_SAMPLES = 25;    
const int MAX_SPM_SAMPLES = 15;        // Holds ~10 SPM readings per 20s block

const float DEADRECONING_DISTANCE_THRESHOLD = 2; // Meters, to filter out GPS noise when stationary

// --- PINS ---
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

struct SpmData {
  uint32_t timestamp;
  float spm;
};

// --- DOUBLE BUFFERS ---
ImuData imuBuffer[2][MAX_IMU_SAMPLES];
GpsData gpsBuffer[2][MAX_GPS_SAMPLES];
SpmData spmBuffer[2][MAX_SPM_SAMPLES];

volatile int imuCount[2] = {0, 0};
volatile int gpsCount[2] = {0, 0};
volatile int spmCount[2] = {0, 0};
volatile uint8_t activeBank = 0; 

// --- OBJECTS ---
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));
SdFat sd;
File32 myFile;
Adafruit_MPU6050 mpu;
TwoWire MyWire = TwoWire(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, MPU_SDA, MPU_SCL);
TinyGPSPlus gps;

// --- GLOBAL STATE ---
bool sdOK = false;
bool mpuOK = false;
bool fixFound = false;
char logFileName[16] = "temp.csv";
String errorMessage = ""; 

double startLat = 0, startLon = 0;
double lastLat = 0, lastLon = 0;
double totalDist = 0.0;
unsigned long startTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastDisplayTime = 0; // NEW: Tracks the 2s loop

float lastDispSpm = -1.0;
int lastDispDist = -1;
int lastDispMin = -1;
int lastDispSplitMin = -1;
int lastDispSplitSec = -1;
int lastDispSats = -1;
String lastDispStatus = "";


TaskHandle_t SensorTaskHandle;

// --- HELPER FUNCTIONS ---
void logError(String msg) {
  errorMessage = msg;
  Serial.println(msg);
}

// --- NEW SPM TUNING PARAMS (Matching Python) ---
const float SIGNAL_SMOOTHING_ALPHA = 0.3f;
const float MAX_ROWING_SPM_CUTOFF = 55.0f;
const float MIN_ROWING_SPM_CUTOFF = 10.0f;
const float ALLOWED_DEVIATION = 7.0f;
const float SPM_SMOOTHING_ALPHA = 0.3f;
const float SPEED_GATE = 1.5f; // km/h
const int STRIKE_CONFIRMATION_THRESHOLD = 3;

// --- SPM STATE ---
float spmHistory[4] = {0, 0, 0, 0};
int historyCount = 0;
int confirmationCounter = 0;
float currentSmoothedSpm = 0;
float prevRawMag = 0; // For signal pre-smoothing

float calculateStrokeRate() {
    // 1. GPS STATIONARY GATE
    // Check current speed and recent distance against your thresholds
    float currentSpeed = gps.speed.kmph();
    // We check if we are actually "Moving" based on your logic
    bool isMoving = (currentSpeed > SPEED_GATE); 
    // Note: If speed is low, the main loop logic for totalDist handles the 4m/5m check

    if (!isMoving && currentSpeed < 0.5f) {
        confirmationCounter = 0;
        currentSmoothedSpm = 0;
        return 0;
    }

    int countActive = imuCount[activeBank];
    int neededSamples = 350; // 7 seconds at 50Hz (Matching WINDOW_SEC = 7.0)
    
    float mag[350];
    int magIdx = 0;
    float sum = 0;

    // 2. DATA ACQUISITION (Cross-Bank)
    uint8_t banks[2] = {!activeBank, activeBank};
    for (int b = 0; b < 2; b++) {
        uint8_t bank = banks[b];
        int count = imuCount[bank];
        int start = (b == 0) ? max(0, count - (neededSamples - countActive)) : 0;
        int end = (b == 0) ? count : min(count, neededSamples - magIdx);

        for (int i = start; i < end; i++) {
            float ax = imuBuffer[bank][i].ax / 100.0f;
            float ay = imuBuffer[bank][i].ay / 100.0f;
            float az = imuBuffer[bank][i].az / 100.0f;
            float rawMag = sqrt(ax*ax + ay*ay + az*az) - 9.81f;

            // --- SIGNAL PRE-SMOOTHING ---
            float smoothedMag = (SIGNAL_SMOOTHING_ALPHA * rawMag) + ((1.0f - SIGNAL_SMOOTHING_ALPHA) * prevRawMag);
            prevRawMag = smoothedMag;
            
            mag[magIdx] = smoothedMag;
            sum += smoothedMag;
            magIdx++;
            if (magIdx >= neededSamples) break;
        }
    }

    if (magIdx < 250) return 0; // Need at least 5s of data

    // 3. CENTER SIGNAL
    float mean = sum / magIdx;
    for (int i = 0; i < magIdx; i++) mag[i] -= mean;

    // 4. AUTOCORRELATION (Search window for 10 SPM to 100 SPM)
    // 10 SPM = 300 lags, 100 SPM = 30 lags at 50Hz
    int min_lag = 30;  
    int max_lag = 300; 
    
    float best_corr = -999999.0f;
    int best_lag = min_lag;
    float corr_sum = 0;

    for (int lag = min_lag; lag <= max_lag; lag++) {
        float corr = 0;
        for (int i = 0; i < magIdx - lag; i++) {
            corr += mag[i] * mag[i + lag];
        }
        corr_sum += corr;
        if (corr > best_corr) {
            best_corr = corr;
            best_lag = lag;
        }
    }

    // 5. VALIDATION & CONFIRMATION
    float avg_corr = corr_sum / (max_lag - min_lag + 1);
    float rawSpm = 0;

    if (best_corr > avg_corr * 1.2f) {
        rawSpm = 60.0f / (best_lag * 0.02f);
    }

    // Hard Cutoffs
    if (rawSpm > MAX_ROWING_SPM_CUTOFF || rawSpm < MIN_ROWING_SPM_CUTOFF) {
        rawSpm = 0;
    }

    // --- ACTIVATION DELAY ---
    if (rawSpm > 0) {
        confirmationCounter++;
    } else {
        confirmationCounter = 0;
    }

    if (confirmationCounter < STRIKE_CONFIRMATION_THRESHOLD) {
        currentSmoothedSpm = 0;
        return 0;
    }

    // 6. ADAPTIVE HISTORY FILTER
    float acceptedSpm = rawSpm;
    if (historyCount >= 3) {
        float historyAvg = 0;
        for (int i = 0; i < 4; i++) historyAvg += spmHistory[i];
        historyAvg /= 4.0f;

        if (abs(rawSpm - historyAvg) > ALLOWED_DEVIATION) {
            acceptedSpm = historyAvg; // Reject jump, use average
        }
    }

    // Update History Circular Buffer
    spmHistory[historyCount % 4] = acceptedSpm;
    historyCount++;

    // 7. FINAL METRIC SMOOTHING (EMA)
    if (acceptedSpm == 0 || currentSmoothedSpm == 0) {
        currentSmoothedSpm = acceptedSpm; // Instant snap for starts/stops
    } else {
        currentSmoothedSpm = (SPM_SMOOTHING_ALPHA * acceptedSpm) + ((1.0f - SPM_SMOOTHING_ALPHA) * currentSmoothedSpm);
    }

    return currentSmoothedSpm;
}

// ... [drawSetupScreen and drawMainScreen remain exactly the same] ...
// Draw the Setup Screen
void drawSetupScreen(String timeStr) {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(5, 30); display.setTextSize(2); display.print("SpeedCoach");
    display.setCursor(5, 60); display.setTextSize(3); display.print(timeStr);
    display.setTextSize(1);
    display.setCursor(5, 90); display.print("SD Card: "); if(sdOK) display.print("OK"); else display.print("ERROR");
    display.setCursor(5, 105); display.print("Sensor: "); if(mpuOK) display.print("OK"); else display.print("ERROR");
    display.setCursor(5, 140); display.setTextSize(2); display.print("Finding Sats...");
  } while (display.nextPage());
}

// Call this ONCE when the GPS fix is found and recording starts
void drawBackground() {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    
    // Top Bar Line
    display.drawLine(0, 26, display.width(), 26, GxEPD_BLACK);
    
    // 4-Quadrant Grid
    int midX = display.width() / 2;
    int midY = ((display.height() - 26) / 2) + 26;
    display.drawLine(midX, 26, midX, display.height(), GxEPD_BLACK); // Vertical
    display.drawLine(0, midY, display.width(), midY, GxEPD_BLACK);   // Horizontal

    // Labels (Static)
    display.setTextSize(1);
    display.setCursor(5, 30); display.print("Split");
    display.setCursor(midX + 5, 30); display.print("Stroke Rate");
    display.setCursor(5, midY + 5); display.print("Distance");
    display.setCursor(midX + 5, midY + 5); display.print("Time");
    
    // Units (Static)
    display.setCursor(midX - 35, 30); display.print("/500m");
    display.setCursor(display.width() - 25, 30); display.print("spm");
    display.setCursor(midX - 15, midY + 5); display.print("m");
    display.setCursor(display.width() - 25, midY + 5); display.print("min");

  } while (display.nextPage());
}

void drawMainScreen(float avgSpeed, float dist, float minutes, int sats, float currentSpm) {
  // 1. Calculations
  int splitMin = 0, splitSec = 0;
  if (avgSpeed > 0.5) {
      float totalSecondsFor500m = 1800.0 / avgSpeed; 
      splitMin = (int)(totalSecondsFor500m / 60);
      splitSec = (int)(totalSecondsFor500m) % 60;
  }
  String status = (currentSpm > 0) ? "ROWING" : "STATIONARY";
  
  int midX = display.width() / 2;
  int midY = ((display.height() - 25) / 2) + 25;

  // --- SMART UPDATES ---

  // Top Bar (Avoids the horizontal line at Y=26)
  if (sats != lastDispSats || status != lastDispStatus) {
      char timeBuf[6];
      sprintf(timeBuf, "%02d:%02d", gps.time.hour()+2, gps.time.minute());  // Adjust to Stockholm
      
      display.setPartialWindow(0, 0, display.width(), 20); // Height 20 doesn't touch line at 26
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextSize(1);
        display.setCursor(5, 8); display.print("Sats: "); display.print(sats);
        display.setCursor(display.width() - 40, 8); display.print(timeBuf);
        
        int16_t x1, y1; uint16_t w, h;
        display.setTextSize(2);
        display.getTextBounds(status, 0, 0, &x1, &y1, &w, &h);
        display.setCursor((display.width()/2)-(w/2), 6); display.print(status);
      } while (display.nextPage());
      lastDispSats = sats; lastDispStatus = status;
  }

  // Split Area (Centered in Top-Left)
  if (splitMin != lastDispSplitMin || splitSec != lastDispSplitSec) {
      // Window is pulled away from center lines and labels
      display.setPartialWindow(0, 42, midX - 20, 40); 
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextSize(3);
        display.setCursor(10, 50); // Centered more to the right
        if (avgSpeed > 0.5) {
            display.print(splitMin); display.print(":");
            if(splitSec < 10) display.print("0");
            display.print(splitSec);
        } else { display.print("--:--"); }
      } while (display.nextPage());
      lastDispSplitMin = splitMin; lastDispSplitSec = splitSec;
  }

  // Stroke Rate (Centered in Top-Right)
  if ((int)currentSpm != (int)lastDispSpm) {
      display.setPartialWindow(midX + 10, 42, midX - 45, 40); // Pulled back from "spm" unit
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextSize(4);
        display.setCursor(midX + 20, 50);
        display.print((int)currentSpm);
      } while (display.nextPage());
      lastDispSpm = (int)currentSpm;
  }

  // Distance (Centered in Bottom-Left)
  if ((int)dist != lastDispDist) {
      display.setPartialWindow(10, midY + 23, midX - 35, 40); // Pulled back from "m" unit
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextSize(4);
        display.setCursor(15, midY + 30);
        display.print((int)dist);
      } while (display.nextPage());
      lastDispDist = (int)dist;
  }

  // Time/Duration (Centered in Bottom-Right)
  if ((int)minutes != lastDispMin) {
      display.setPartialWindow(midX + 10, midY + 23, midX - 50, 40); // Pulled back from "min" unit
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextSize(4);
        display.setCursor(midX + 20, midY + 30);
        display.print((int)minutes);
      } while (display.nextPage());
      lastDispMin = (int)minutes;
  }
}


void flushDataToSD(uint8_t bankToRead) {
  logError("SD Write Success");
  if (!sdOK) return;

  if (myFile.open(logFileName, O_RDWR | O_CREAT | O_APPEND)) {
    // 1. Write GPS Chunk
    for (int i = 0; i < gpsCount[bankToRead]; i++) {
      myFile.print("GPS,");
      myFile.print(gpsBuffer[bankToRead][i].timestamp); myFile.print(",");
      myFile.print(gpsBuffer[bankToRead][i].lat, 6); myFile.print(",");
      myFile.print(gpsBuffer[bankToRead][i].lon, 6); myFile.print(",");
      myFile.print(gpsBuffer[bankToRead][i].speed); myFile.print(",");
      myFile.print(gpsBuffer[bankToRead][i].distToStart); myFile.print(",");
      myFile.println(gpsBuffer[bankToRead][i].sats);
    }
    // 2. Write IMU Chunk
    for (int i = 0; i < imuCount[bankToRead]; i++) {
      myFile.print("IMU,");
      myFile.print(imuBuffer[bankToRead][i].timestamp); myFile.print(",");
      myFile.print(imuBuffer[bankToRead][i].ax); myFile.print(",");
      myFile.print(imuBuffer[bankToRead][i].ay); myFile.print(",");
      myFile.print(imuBuffer[bankToRead][i].az); myFile.print(",");
      myFile.print(imuBuffer[bankToRead][i].gx); myFile.print(",");
      myFile.print(imuBuffer[bankToRead][i].gy); myFile.print(",");
      myFile.println(imuBuffer[bankToRead][i].gz);
    }
    // 3. Write SPM Chunk (All the 2-second updates that happened in the last 20s)
    for (int i = 0; i < spmCount[bankToRead]; i++) {
      myFile.print("SPM,");
      myFile.print(spmBuffer[bankToRead][i].timestamp); myFile.print(",");
      myFile.println(spmBuffer[bankToRead][i].spm, 2);
    }
    myFile.close();
  } else {
    logError("SD Write Fail");
    sdOK = false; 
  }
}

// =========================================================================
//    BACKGROUND SENSOR TASK (Unchanged, Priority 2)
// =========================================================================
void SensorTask(void *pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(IMU_INTERVAL_MS); 

  for (;;) {
    unsigned long currentMillis = millis();
    uint8_t bank = activeBank;

    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        if (gps.location.isUpdated() && fixFound) {
          double distStep = gps.distanceBetween(gps.location.lat(), gps.location.lng(), lastLat, lastLon);
          if (distStep > DEADRECONING_DISTANCE_THRESHOLD) {
            totalDist += distStep;
            lastLat = gps.location.lat();
            lastLon = gps.location.lng();
          }

          if (gpsCount[bank] < MAX_GPS_SAMPLES) {
            int idx = gpsCount[bank];
            gpsBuffer[bank][idx].timestamp = currentMillis;
            gpsBuffer[bank][idx].lat = gps.location.lat();
            gpsBuffer[bank][idx].lon = gps.location.lng();
            gpsBuffer[bank][idx].speed = gps.speed.kmph();
            gpsBuffer[bank][idx].distToStart = gps.distanceBetween(gps.location.lat(), gps.location.lng(), startLat, startLon);
            gpsBuffer[bank][idx].sats = gps.satellites.value();
            gpsCount[bank]++;
          }
        }
      }
    }

    if (mpuOK && fixFound && imuCount[bank] < MAX_IMU_SAMPLES) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      int idx = imuCount[bank];
      imuBuffer[bank][idx].timestamp = currentMillis;
      imuBuffer[bank][idx].ax = (int16_t)(a.acceleration.x * 100);
      imuBuffer[bank][idx].ay = (int16_t)(a.acceleration.y * 100);
      imuBuffer[bank][idx].az = (int16_t)(a.acceleration.z * 100);
      imuBuffer[bank][idx].gx = (int16_t)(g.gyro.x * 100);
      imuBuffer[bank][idx].gy = (int16_t)(g.gyro.y * 100);
      imuBuffer[bank][idx].gz = (int16_t)(g.gyro.z * 100);
      imuCount[bank]++;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
// =========================================================================

// --- SETUP ---
void setup() {
  Serial.begin(115200);

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
  display.setRotation(3);

  drawSetupScreen("--:--");

  xTaskCreate(SensorTask, "Sensors", 2048, NULL, 2, &SensorTaskHandle);

  while (!fixFound) {
    if (gps.location.isValid() && gps.date.year() > 2000) {
      fixFound = true;
      startLat = gps.location.lat();
      startLon = gps.location.lng();
      lastLat = startLat;
      lastLon = startLon;
      startTime = millis();
      lastLogTime = millis();
      lastDisplayTime = millis();

      if (sdOK) {
        sprintf(logFileName, "%02d%02d_%02d%02d.csv", gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute());
        if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
          myFile.println("Type,Millis,Data1,Data2,Data3,Data4,Data5,Data6");
          myFile.close();
        } else { sdOK = false; }
      }
    }
    delay(10); 
  }

  drawBackground();
}

// --- MAIN LOOP ---
void loop() {
  unsigned long currentMillis = millis();

  // --- EVERY 2 SECONDS: Calculate SPM and Update Display ---
  if (currentMillis - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    Serial.println("Updating display...");
    lastDisplayTime = currentMillis;

    // 1. Calculate Stroke Rate (Takes < 2ms)
    float newSpm = calculateStrokeRate();

    // 2. Save it to the current bank's SPM buffer for later SD writing
    if (spmCount[activeBank] < MAX_SPM_SAMPLES) {
      int idx = spmCount[activeBank];
      spmBuffer[activeBank][idx].timestamp = currentMillis;
      spmBuffer[activeBank][idx].spm = newSpm;
      spmCount[activeBank]++;
    }

    // 3. Calculate Speed Averages (Last 10s of GPS data from active bank)
    float speedSum = 0;
    int speedSamples = 0;
    unsigned long cutoffTime = currentMillis - 10000; 

    for (int i = 0; i < gpsCount[activeBank]; i++) {
      if (gpsBuffer[activeBank][i].timestamp >= cutoffTime) {
        speedSum += gpsBuffer[activeBank][i].speed;
        speedSamples++;
      }
    }
    float avgSpeed10s = (speedSamples > 0) ? (speedSum / speedSamples) : 0.0;
    float totalTimeMin = (currentMillis - startTime) / 60000.0;

    // 4. Draw to E-Paper Display
    drawMainScreen(avgSpeed10s, totalDist, totalTimeMin, gps.satellites.value(), newSpm);
  }

  // --- EVERY 20 SECONDS: Swap Buffers and Save to SD ---
  if (currentMillis - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentMillis;

    // 1. SWAP BUFFERS
    uint8_t processBank = activeBank; 
    activeBank = !activeBank;         

    // 2. Flush data to SD Card
    flushDataToSD(processBank);

    // 3. Clear the processed bank to make it ready for sensors again
    imuCount[processBank] = 0;
    gpsCount[processBank] = 0;
    spmCount[processBank] = 0;
  }
  
  delay(50); 
}