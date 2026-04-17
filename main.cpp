#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>

// ==========================================
//             --- CONFIGURATION ---
// ==========================================

// Data Logging & Memory
const int BUFFER_SIZE = 20;               // Number of readings to hold before writing to SD
const char* LOG_FILE_FORMAT = "%02d.%02d.@%02d:%02d.csv"; // Format: Month, Day, Hour, Minute
const int SD_SPI_SPEED_MHZ = 12;          // SPI speed for SD card communication

// GPS & Movement Logic
const double MIN_MOVEMENT_DIST_M = 1.0;   // Minimum distance (meters) to register movement ("dead reckoning" filter)
const int AVG_WINDOW = 10;                // Number of readings used for rolling average speed
const uint32_t GPS_BAUD_RATE = 9600;      // Baud rate for GPS serial connection

// Display & UI
const int UI_REFRESH_SETUP_MS = 2000;     // Setup screen refresh rate (milliseconds)
const int EPD_ROTATION = 3;               // E-ink rotation (0-3). 3 is usually landscape
const uint32_t SERIAL_BAUD_RATE = 115200; // General Serial & Display init baud rate

// IMU (MPU6050) Settings
#define MPU_ACCEL_RANGE MPU6050_RANGE_8_G
#define MPU_GYRO_RANGE  MPU6050_RANGE_500_DEG
#define MPU_FILTER_BW   MPU6050_BAND_21_HZ

// ==========================================

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
struct RowData {
  unsigned long time;
  double lat;
  double lon;
  float speed;      
  float distToStart;
  int sats;
};

RowData dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;

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

double totalDist = 0.0;
unsigned long startTime = 0;
double startLat = 0, startLon = 0;
double lastLat = 0, lastLon = 0;

// --- DRAWING HELPERS ---
void drawIcons(int sats) {
  int topY = 4; 
  int rightEdge = display.width() - 2; 
  
  // --- 1. GPS BARS ---
  int barsFilled = 0;
  if (sats > 3) barsFilled = 1;
  if (sats > 5) barsFilled = 2;
  if (sats > 7) barsFilled = 3;
  if (sats > 9) barsFilled = 4;

  for(int i=0; i<4; i++) {
    int h = (i+1)*3; 
    int x = rightEdge - 18 + (i*5);
    int y = topY + (12 - h);
    
    display.drawRect(x, y, 3, h, GxEPD_BLACK);
    if (i < barsFilled) {
      display.fillRect(x, y, 3, h, GxEPD_BLACK);
    }
  }
  
  display.setTextSize(1);
  display.setCursor(rightEdge - 30, topY + 4);
  display.print(sats);

  // --- 2. GYRO ICON ---
  int gyroX = rightEdge - 50;
  if (mpuOK) {
    display.drawRect(gyroX, topY, 12, 12, GxEPD_BLACK);     
    display.drawRect(gyroX+4, topY+4, 4, 4, GxEPD_BLACK);   
    display.drawLine(gyroX+6, topY, gyroX+6, topY+2, GxEPD_BLACK);   
    display.drawLine(gyroX+6, topY+10, gyroX+6, topY+12, GxEPD_BLACK); 
    display.drawLine(gyroX, topY+6, gyroX+2, topY+6, GxEPD_BLACK);     
    display.drawLine(gyroX+10, topY+6, gyroX+12, topY+6, GxEPD_BLACK); 
  } else {
    display.drawRect(gyroX, topY, 12, 12, GxEPD_BLACK);
    display.drawLine(gyroX, topY, gyroX+12, topY+12, GxEPD_BLACK); 
  }

  // --- 3. SD CARD ICON ---
  int sdX = rightEdge - 70;
  if (sdOK) {
    display.drawRect(sdX, topY+2, 11, 10, GxEPD_BLACK); 
    display.drawLine(sdX, topY+2, sdX, topY, GxEPD_BLACK); 
    display.drawLine(sdX, topY, sdX+7, topY, GxEPD_BLACK); 
    display.drawLine(sdX+7, topY, sdX+10, topY+3, GxEPD_BLACK); 
    
    display.fillRect(sdX+2, topY+9, 2, 2, GxEPD_BLACK);
    display.fillRect(sdX+5, topY+9, 2, 2, GxEPD_BLACK);
    display.fillRect(sdX+8, topY+9, 2, 2, GxEPD_BLACK);
  } else {
    display.setCursor(sdX, topY+4);
    display.print("!SD");
  }
}

void drawMainScreen(int hour, int minute, float avgSpeed10s, float totalDist, float totalTimeMin) {
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    // --- HEADER ---
    int headerH = 24;
    display.drawLine(0, headerH, display.width(), headerH, GxEPD_BLACK);

    display.setTextSize(2);
    display.setCursor(2, 4); 
    if(hour < 10) display.print("0");
    display.print(hour);
    display.print(":");
    if(minute < 10) display.print("0");
    display.print(minute);

    drawIcons(gps.satellites.value());

    // --- GRID LAYOUT ---
    int contentH = display.height() - headerH;
    int midY = headerH + (contentH / 2);
    int midX = display.width() / 2;

    display.drawLine(midX, headerH, midX, display.height(), GxEPD_BLACK); 
    display.drawLine(0, midY, display.width(), midY, GxEPD_BLACK);        

    // --- QUADRANT 1: AVG SPEED ---
    display.setCursor(5, headerH + 5);
    display.setTextSize(1); display.println("AVG (10s)");
    
    display.setCursor(5, headerH + 20);
    display.setTextSize(3); display.print(avgSpeed10s, 1);
    display.setTextSize(1); display.print(" km/h");

    // --- QUADRANT 2: STROKE RATE ---
    display.setCursor(midX + 5, headerH + 5);
    display.setTextSize(1); display.println("STROKE/M");
    
    display.setCursor(midX + 5, headerH + 20);
    display.setTextSize(3); display.print("0.0"); // Ready for IMU logic
    display.setTextSize(1); display.print(" s/m");

    // --- QUADRANT 3: DISTANCE ---
    display.setCursor(5, midY + 5);
    display.setTextSize(1); display.println("DIST");
    
    display.setCursor(5, midY + 20);
    display.setTextSize(3); display.print((int)totalDist);
    display.setTextSize(1); display.print(" m");

    // --- QUADRANT 4: TIME ---
    display.setCursor(midX + 5, midY + 5);
    display.setTextSize(1); display.println("TIME");
    
    display.setCursor(midX + 5, midY + 20);
    display.setTextSize(3); display.print((int)totalTimeMin);
    display.setTextSize(1); display.print(" min");

  } while (display.nextPage());
}

void flushBufferToSD() {
  if (!sdOK) return;
  if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
      myFile.print(dataBuffer[i].time); myFile.print(",");
      myFile.print(dataBuffer[i].lat, 6); myFile.print(",");
      myFile.print(dataBuffer[i].lon, 6); myFile.print(",");
      myFile.print(dataBuffer[i].speed); myFile.print(",");
      myFile.print(dataBuffer[i].distToStart); myFile.print(",");
      myFile.println(dataBuffer[i].sats);
    }
    myFile.close();
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  
  MyWire.begin();
  mpuOK = mpu.begin(MPU6050_I2CADDR_DEFAULT, &MyWire);
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU_ACCEL_RANGE);
    mpu.setGyroRange(MPU_GYRO_RANGE);
    mpu.setFilterBandwidth(MPU_FILTER_BW);
  }

  Serial1.setPins(GPS_RX_PIN, GPS_TX_PIN);
  Serial1.begin(GPS_BAUD_RATE);

  SPI.setPins(PIN_MISO, PIN_SCK, PIN_MOSI);
  sdOK = sd.begin(SD_CS, SD_SCK_MHZ(SD_SPI_SPEED_MHZ));

  display.init(SERIAL_BAUD_RATE); 
  display.setRotation(EPD_ROTATION); 

  // --- WAITING FOR GPS SCREEN ---
  unsigned long lastUpdate = 0;
  
  while (!fixFound) {
    while (Serial1.available() > 0) gps.encode(Serial1.read());

    if (gps.location.isValid()) {
      fixFound = true;
      startLat = gps.location.lat();
      startLon = gps.location.lng();
      lastLat = startLat;
      lastLon = startLon;
      startTime = millis();
      
      if (sdOK) {
        sprintf(logFileName, LOG_FILE_FORMAT, 
               gps.date.month(), gps.date.day(), 
               gps.time.hour(), gps.time.minute());
        if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
          myFile.println("Millis,Lat,Lon,Speed,DistStart,Sats");
          myFile.close();
        }
      }
    }

    // Refresh Setup Screen
    if (millis() - lastUpdate > UI_REFRESH_SETUP_MS && !fixFound) {
      lastUpdate = millis();
      display.setFullWindow();
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        
        display.drawLine(0, 30, display.width(), 30, GxEPD_BLACK);
        display.setCursor(10, 10);
        display.setTextSize(2);
        display.print("OpenSpeedCoach");

        int yStart = 40;
        int rowH = 25;
        
        display.setTextSize(1);
        
        display.setCursor(10, yStart);
        display.print("SD Card: "); 
        if(sdOK) display.println("OK"); else display.println("MISSING");

        display.setCursor(10, yStart + rowH);
        display.print("Sensor:  "); 
        if(mpuOK) display.println("OK"); else display.println("ERROR");

        display.setCursor(10, yStart + (rowH*2));
        display.print("Sats:    "); 
        display.print(gps.satellites.value());
        if(gps.satellites.value() == 0) display.print(" (Search...)");
        else display.print(" (Locking...)");

      } while (display.nextPage());
    }
  }
}

// --- MAIN LOOP ---
void loop() {
  while (Serial1.available() > 0) gps.encode(Serial1.read());

  if (gps.location.isUpdated()) {
    
    double distStep = gps.distanceBetween(gps.location.lat(), gps.location.lng(), lastLat, lastLon);
    if (distStep > MIN_MOVEMENT_DIST_M) {
      totalDist += distStep;
      lastLat = gps.location.lat();
      lastLon = gps.location.lng();
    }

    if (bufferIndex < BUFFER_SIZE) {
      dataBuffer[bufferIndex].time = millis();
      dataBuffer[bufferIndex].lat = gps.location.lat();
      dataBuffer[bufferIndex].lon = gps.location.lng();
      dataBuffer[bufferIndex].speed = gps.speed.kmph();
      dataBuffer[bufferIndex].distToStart = gps.distanceBetween(gps.location.lat(), gps.location.lng(), startLat, startLon);
      dataBuffer[bufferIndex].sats = gps.satellites.value();
      bufferIndex++;
    }

    if (bufferIndex >= BUFFER_SIZE) {
      float sumSpeed = 0;
      int count = 0;
      for (int i = BUFFER_SIZE - AVG_WINDOW; i < BUFFER_SIZE; i++) {
        if (i >= 0) {
          sumSpeed += dataBuffer[i].speed;
          count++;
        }
      }
      float avgSpeed10s = (count > 0) ? (sumSpeed / count) : 0.0;
      float totalTimeMin = (millis() - startTime) / 60000.0;

      flushBufferToSD();
      drawMainScreen(gps.time.hour(), gps.time.minute(), avgSpeed10s, totalDist, totalTimeMin);
      bufferIndex = 0;
    }
  }
}