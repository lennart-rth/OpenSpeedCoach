#ifndef DATATYPES_H
#define DATATYPES_H

#include <Arduino.h>

// --- CONFIGURATION ---
const int LOG_INTERVAL_MS = 10000;     
const int DISPLAY_INTERVAL_MS = 3000;  
const int IMU_INTERVAL_MS = 10;        // 100Hz

const int MAX_IMU_SAMPLES = 1200;     
const int MAX_GPS_SAMPLES = 25;        
const int MAX_SPM_SAMPLES = 15;        

const float DEADRECONING_DISTANCE_THRESHOLD = 2.0; 

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
  float hdop;
  float altitude;
  float course;
};

struct SpmData {
  uint32_t timestamp;
  float spm;
};

#endif