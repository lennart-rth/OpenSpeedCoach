#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>

// --- CONFIGURATION ---
const int BUFFER_SIZE = 20;      
const int AVG_WINDOW = 10;       

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
// NOTE: If rotation is still wrong, try changing '3' to '0' or '2' in setup()
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
double lastLat = 0, lastLon = 0;


// --- DRAWING HELPERS ---

void drawIcons(int sats) {
  // Fixed positions for the top right corner
  int topY = 4; 
  int rightEdge = display.width() - 2; // Rightmost pixel
  
  // --- 1. GPS BARS (Far Right) ---
  // Bars are 3px wide, spaced 2px apart.
  // 4 Bars total.
  // X positions: rightEdge - 20, -15, -10, -5
  
  int barsFilled = 0;
  if (sats > 3) barsFilled = 1;
  if (sats > 5) barsFilled = 2;
  if (sats > 7) barsFilled = 3;
  if (sats > 9) barsFilled = 4;

  for(int i=0; i<4; i++) {
    int h = (i+1)*3; // Heights: 3, 6, 9, 12
    int x = rightEdge - 18 + (i*5);
    int y = topY + (12 - h);
    
    // Always draw the Outline (Hollow)
    display.drawRect(x, y, 3, h, GxEPD_BLACK);
    
    // If signal is strong enough, Fill it
    if (i < barsFilled) {
      display.fillRect(x, y, 3, h, GxEPD_BLACK);
    }
  }
  
  // Sat Count (Small number to the left of bars)
  display.setTextSize(1);
  display.setCursor(rightEdge - 30, topY + 4);
  display.print(sats);

  // --- 2. GYRO ICON (Middle) ---
  // A "Chip" symbol: Square with a crosshair
  int gyroX = rightEdge - 50;
  if (mpuOK) {
    display.drawRect(gyroX, topY, 12, 12, GxEPD_BLACK);     // Outer Box
    display.drawRect(gyroX+4, topY+4, 4, 4, GxEPD_BLACK);   // Inner Box
    display.drawLine(gyroX:q+6, topY, gyroX+6, topY+2, GxEPD_BLACK);   // Top Pin
    display.drawLine(gyroX+6, topY+10, gyroX+6, topY+12, GxEPD_BLACK); // Bottom Pin
    display.drawLine(gyroX, topY+6, gyroX+2, topY+6, GxEPD_BLACK);     // Left Pin
    display.drawLine(gyroX+10, topY+6, gyroX+12, topY+6, GxEPD_BLACK); // Right Pin
  } else {
    // Cross out if missing
    display.drawRect(gyroX, topY, 12, 12, GxEPD_BLACK);
    display.drawLine(gyroX, topY, gyroX+12, topY+12, GxEPD_BLACK); 
  }

  // --- 3. SD CARD ICON (Left) ---
  // Shape: Rectangle with notched top-right corner
  int sdX = rightEdge - 70;
  if (sdOK) {
    // Main Body
    display.drawRect(sdX, topY+2, 11, 10, GxEPD_BLACK); // Bottom part
    display.drawLine(sdX, topY+2, sdX, topY, GxEPD_BLACK); // Left wall up
    display.drawLine(sdX, topY, sdX+7, topY, GxEPD_BLACK); // Top wall
    display.drawLine(sdX+7, topY, sdX+10, topY+3, GxEPD_BLACK); // Diagonal notch
    
    // Contacts (Pins)
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

    // --- HEADER (Fixed Height: 24px) ---
    // Divider Line (Further up now)
    int headerH = 24;
    display.drawLine(0, headerH, display.width(), headerH, GxEPD_BLACK);

    // Time (Top Left)
    display.setTextSize(2);
    display.setCursor(2, 4); // Tucked in top-left
    if(hour < 10) display.print("0");
    display.print(hour);
    display.print(":");
    if(minute < 10) display.print("0");
    display.print(minute);

    // Icons (Top Right)
    drawIcons(gps.satellites.value());

    // --- GRID LAYOUT ---
    // Calculate space remaining below header
    int contentH = display.height() - headerH;
    int midY = headerH + (contentH / 2);
    int midX = display.width() / 2;

    // Grid Lines
    display.drawLine(midX, headerH, midX, display.height(), GxEPD_BLACK); // Vertical Center
    display.drawLine(0, midY, display.width(), midY, GxEPD_BLACK);        // Horizontal Center

    // --- QUADRANT 1: AVG SPEED (Top Left) ---
    display.setCursor(5, headerH + 5);
    display.setTextSize(1); display.println("AVG (10s)");
    
    display.setCursor(5, headerH + 20);
    display.setTextSize(3); display.print(avgSpeed10s, 1);
    display.setTextSize(1); display.print(" km/h");

    // --- QUADRANT 2: STROKE RATE (Top Right) ---
    display.setCursor(midX + 5, headerH + 5);
    display.setTextSize(1); display.println("STROKE/M");
    
    display.setCursor(midX + 5, headerH + 20);
    display.setTextSize(3); display.print("0.0"); // Dummy
    display.setTextSize(1); display.print(" s/m");

    // --- QUADRANT 3: DISTANCE (Bottom Left) ---
    display.setCursor(5, midY + 5);
    display.setTextSize(1); display.println("DIST");
    
    display.setCursor(5, midY + 20);
    display.setTextSize(3); display.print((int)totalDist);
    display.setTextSize(1); display.print(" m");

    // --- QUADRANT 4: TIME (Bottom Right) ---
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
  
  // FIX: Rotation 3 usually sets Landscape with buttons at bottom for these HATs
  display.setRotation(3); 

  // --- WAITING FOR GPS SCREEN ---
  unsigned long lastUpdate = 0;
  
  while (!fixFound) {
    while (Serial1.available() > 0) gps.encode(Serial1.read());

    if (gps.location.isValid() && gps.date.year() > 2000) {
      fixFound = true;
      startLat = gps.location.lat();
      startLon = gps.location.lng();
      lastLat = startLat;
      lastLon = startLon;
      startTime = millis();
      
      if (sdOK) {
        sprintf(logFileName, "%02d%02d%02d%02d.csv", 
               gps.date.month(), gps.date.day(), 
               gps.time.hour(), gps.time.minute());
        if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
          myFile.println("Millis,Lat,Lon,Speed,DistStart,Sats");
          myFile.close();
        }
      }
    }

    // Refresh Setup Screen
    if (millis() - lastUpdate > UI_REFRESH_SETUP && !fixFound) {
      lastUpdate = millis();
      display.setFullWindow();
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        
        // Header
        display.drawLine(0, 30, display.width(), 30, GxEPD_BLACK);
        display.setCursor(10, 20);
        display.setTextSize(2);
        display.print("OpenSpeedCoach");

        // Status List
        int yStart = 50;
        int rowH = 25;
        
        display.setTextSize(1);
        
        // SD Status
        display.setCursor(10, yStart);
        display.print("SD Card: "); 
        if(sdOK) display.println("OK"); else display.println("MISSING");

        // MPU Status
        display.setCursor(10, yStart + rowH);
        display.print("Sensor:  "); 
        if(mpuOK) display.println("OK"); else display.println("ERROR");

        // GPS Status
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
    if (distStep > 2.0) {
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
