#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>

// --- PIN DEFINITIONS ---

// 1. SD CARD PIN
// We use Pin D6 (P0.06) for the SD Card Chip Select
#define SD_CS    PIN_020 

// 2. E-PAPER PINS (Your definitions)
#define EPD_CS   PIN_024
#define EPD_DC   PIN_022
#define EPD_RST  PIN_104
#define EPD_BUSY PIN_106 

// 3. SPI PINS (CRITICAL HARDWARE PINS)
// The SD card strictly needs these specific pins on the Nice!Nano
#define PIN_MISO PIN_017  // P1.11 (Data FROM SD card)
#define PIN_MOSI PIN_011  // P0.10 (Data TO SD card)
#define PIN_SCK  PIN_100  // P0.17 (Clock)

#define MPU_SDA  PIN_029
#define MPU_SCL  PIN_031 

#define GPS_RX_PIN PIN_115
#define GPS_TX_PIN PIN_002 

// --- DISPLAY SELECTION ---
GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display(GxEPD2_270(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

// Create SdFat objects
SdFat sd;
File myFile;

Adafruit_MPU6050 mpu;
TwoWire MyWire = TwoWire(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, MPU_SDA, MPU_SCL);

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  
  // Wait for USB (Clone Fix)
  while (!Serial && millis() < 5000) { delay(10); }

  Serial.println("--- Nice!Nano E-Paper + SD Test ---");

  MyWire.begin();

  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &MyWire)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial1.setPins(GPS_RX_PIN, GPS_TX_PIN);
  Serial1.begin(9600);

  // --- 1. CONFIGURE SPI ---
  // We MUST set the correct hardware pins before starting SD or Display.
  // MISO (43) is required for the SD card to talk back.
  SPI.setPins(PIN_MISO, PIN_SCK, PIN_MOSI);
  
  // 2. Initialize SD Card
  // We use SD_SCK_MHZ(12) for safe speed.
  Serial.print("Initializing SdFat... ");
  if (!sd.begin(SD_CS)) {
    Serial.println("FAILED!");
    // sd.initErrorHalt(); // Helpful for deep debugging
  } else {
    Serial.println("SUCCESS!");
    
    // Write File
    // O_RDWR = Read/Write, O_CREAT = Create, O_AT_END = Append
    if (!myFile.open("test.txt", FILE_WRITE)) {
      Serial.println("Error opening test.txt");
    } else {
      Serial.print("Writing...");
      myFile.println("Hello from SdFat Fork!");
      myFile.close(); 
      Serial.println("Done.");
    }
  }


  // --- 3. INITIALIZE DISPLAY ---
  display.init(115200);
  Serial.println("Drawing...");

  display.setRotation(1);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.println("Hello from open rower!");
    
    // Show SD Status on Screen
    display.setTextSize(1);
    display.setCursor(10, 50);

    // Check if the file exists to confirm success
    if (sd.exists("test.txt")) {
      display.println("SD Card: OK");
      display.println("Mode: SdFat Fork");
    } else {
      display.println("SD Card: FAILED");
    }
    
    display.setCursor(10, 90);
    display.println("SD uses CS: Pin 6");
    
  } while (display.nextPage());

  Serial.println("Done.");
}

void loop() {
  while (Serial1.available() > 0) gps.encode(Serial1.read());
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 5000) {
    lastLog = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Accel X: "); Serial.print(a.acceleration.x);
    Serial.print(", Y: "); Serial.print(a.acceleration.y);
    Serial.print(", Z: "); Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print(gps.satellites.value()); Serial.println(" satellites");
    if (gps.location.isValid()) {
      Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
      Serial.print(", Lng: "); Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("Location: INVALID");
    }

  }
}