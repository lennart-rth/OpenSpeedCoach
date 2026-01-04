#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>

// --- SETTINGS ---
const unsigned long DISPLAY_INTERVAL = 20000; // Update Screen every 20s

// --- PIN DEFINITIONS ---
#define SD_CS    PIN_020 

// E-Paper
#define EPD_CS   PIN_024
#define EPD_DC   PIN_022
#define EPD_RST  PIN_104
#define EPD_BUSY PIN_106 

// SPI
#define PIN_MISO PIN_017
#define PIN_MOSI PIN_011
#define PIN_SCK  PIN_100

// I2C & GPS
#define MPU_SDA  PIN_029
#define MPU_SCL  PIN_031 
#define GPS_RX_PIN PIN_115
#define GPS_TX_PIN PIN_002 

// --- OBJECTS ---
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

SdFat sd;
File32 myFile; 
Adafruit_MPU6050 mpu;
TwoWire MyWire = TwoWire(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, MPU_SDA, MPU_SCL);
TinyGPSPlus gps;

// --- VARIABLES ---
bool fixFound = false;
bool sdReady = false;
bool logFileCreated = false;       // Track if we have named the file yet
char logFileName[16] = "temp.csv"; // Buffer for dynamic filename

// Navigation Data
double startLat = 0.0, startLon = 0.0;
double lastLat = 0.0, lastLon = 0.0;
double totalDistanceTraveled = 0.0; 
double currentSpeed = 0.0;
double avgSpeed = 0.0;
double distToStart = 0.0;

unsigned long startTime = 0;
unsigned long lastDisplayUpdate = 0;

void setup() {
  Serial.begin(115200);
  // while (!Serial && millis() < 2000) { delay(10); } 

  Serial.println("--- Open Rower GPS Logger ---");

  // 1. Init Hardware
  MyWire.begin();
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &MyWire)) {
    Serial.println("MPU6050 Not Found");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  Serial1.setPins(GPS_RX_PIN, GPS_TX_PIN);
  Serial1.begin(9600);

  SPI.setPins(PIN_MISO, PIN_SCK, PIN_MOSI);
  if (!sd.begin(SD_CS, SD_SCK_MHZ(12))) {
    Serial.println("SD Init Failed!");
    sdReady = false;
  } else {
    Serial.println("SD Init Success. Waiting for GPS time to name file...");
    sdReady = true;
  }

  // 2. Init Display
  display.init(115200); 
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.println("Open SpeedCoach");
    display.setTextSize(1);
    display.setCursor(10, 60);
    display.println("Waiting for GPS Fix...");
  } while (display.nextPage());
}

void loop() {
  // --- 1. FEED GPS DATA ---
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // --- 2. ON NEW GPS DATA (Runs ~1Hz) ---
  if (gps.location.isUpdated()) {
    
    // A. Handle First Fix & File Creation
    if (!fixFound) {
      fixFound = true;
      startLat = gps.location.lat();
      startLon = gps.location.lng();
      lastLat = startLat;
      lastLon = startLon;
      startTime = millis();
      Serial.println("GPS Fix Acquired!");
    }

    // B. Create Dynamic Filename (Only once, when we have valid date/time)
    if (sdReady && !logFileCreated && gps.date.isValid() && gps.time.isValid() && gps.date.year() > 2000) {
       // Format: MMDDHHmm.csv (e.g., 01051230.csv for Jan 5th, 12:30)
       // We use standard 8.3 filename format to be safe
       sprintf(logFileName, "%02d%02d%02d%02d.csv", 
               gps.date.month(), gps.date.day(), 
               gps.time.hour(), gps.time.minute());
       
       Serial.print("Creating Log File: ");
       Serial.println(logFileName);

       // Write Header
       if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
          myFile.println("Millis,Lat,Lon,Speed_kmh,Avg_kmh,Dist_Start_m,Total_Dist_m,Sats");
          myFile.close();
          logFileCreated = true; // Prevents us from resetting the name
       } else {
          Serial.println("Error creating file!");
       }
    }

    // C. Calculate Distance Traveled
    double distStep = gps.distanceBetween(gps.location.lat(), gps.location.lng(), lastLat, lastLon);
    if (distStep > 2.0) { // Filter drift < 2m
      totalDistanceTraveled += distStep;
      lastLat = gps.location.lat();
      lastLon = gps.location.lng();
    }

    // D. Calculate Metrics
    currentSpeed = gps.speed.kmph();
    distToStart = gps.distanceBetween(gps.location.lat(), gps.location.lng(), startLat, startLon);
    
    double hoursElapsed = (millis() - startTime) / 3600000.0;
    if (hoursElapsed > 0.001) {
      avgSpeed = (totalDistanceTraveled / 1000.0) / hoursElapsed;
    }

    // E. LOG TO SD CARD (If file is created)
    if (sdReady && logFileCreated) {
       if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
         myFile.print(millis()); myFile.print(",");
         myFile.print(gps.location.lat(), 6); myFile.print(",");
         myFile.print(gps.location.lng(), 6); myFile.print(",");
         myFile.print(currentSpeed); myFile.print(",");
         myFile.print(avgSpeed); myFile.print(",");
         myFile.print(distToStart); myFile.print(",");
         myFile.print(totalDistanceTraveled); myFile.print(",");
         myFile.println(gps.satellites.value());
         myFile.close(); 
       }
    }
    
    Serial.print("Logged to "); Serial.print(logFileName); 
    Serial.print(" | Spd: "); Serial.println(currentSpeed);
  }

  // --- 3. UPDATE DISPLAY (Runs every 20 Seconds) ---
  if (millis() - lastDisplayUpdate > DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();

    Serial.println("Refreshing E-Paper...");
    
    display.setFullWindow(); 
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);

      if (!fixFound) {
        display.setCursor(5, 30);
        display.setTextSize(2);
        display.println("NO GPS FIX");
        display.setTextSize(1);
        display.setCursor(5, 60);
        display.print("Sats: "); display.println(gps.satellites.value());
      } else {
        // Line 1: Current Speed
        display.setCursor(5, 20);
        display.setTextSize(2);
        display.print("SPD: "); display.print(currentSpeed, 1); display.println(" km/h");

        // Line 2: Avg Speed
        display.setCursor(5, 50);
        display.print("AVG: "); display.print(avgSpeed, 1); display.println(" km/h");

        // Line 3: Distances
        display.setCursor(5, 80);
        display.setTextSize(1);
        display.print("Start Dist: "); display.print(distToStart, 0); display.println(" m");
        
        display.setCursor(5, 95);
        display.print("Total Row:  "); display.print(totalDistanceTraveled, 0); display.println(" m");
        
        // Footer
        display.setCursor(5, 115);
        display.print("Sat:"); display.print(gps.satellites.value());
        
        if(sdReady && logFileCreated) {
           // Show filename on screen so you know it worked
           display.print(" F:"); display.print(logFileName);
        }
      }
    } while (display.nextPage());
    
    Serial.println("Display Refresh Done.");
  }
}