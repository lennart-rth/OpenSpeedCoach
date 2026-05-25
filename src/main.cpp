#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPSPlus.h>
#include <malloc.h>

#include "DataTypes.h"
#include "BLEController.h"
#include "Logger.h"
#include "Display.h"
#include "StrokeDetection.h"
#include "PaceEstimator.h"
#include "CurveAnalyzer.h"

// --- MODULES ---
Logger logger;
DisplayManager displayUI;
StrokeDetection strokeDet;
PaceEstimator paceEstimator;
CurveAnalyzer curveAnalyzer;
BLEController bleController;

Adafruit_MPU6050 mpu;
TwoWire MyWire = TwoWire(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, MPU_SDA, MPU_SCL);
TinyGPSPlus gps;

// --- DOUBLE BUFFERS ---
ImuData imuBuffer[2][MAX_IMU_SAMPLES];
GpsData gpsBuffer[2][MAX_GPS_SAMPLES];
SpmData spmBuffer[2][MAX_SPM_SAMPLES];

volatile int imuCount[2] = {0, 0};
volatile int gpsCount[2] = {0, 0};
volatile int spmCount[2] = {0, 0};
volatile uint8_t activeBank = 0; 

volatile uint32_t lastSensorTaskHeartbeat = 0;

// --- GLOBAL STATE ---
bool mpuOK = false;
bool fixFound = false;
bool gpsCommEverSeen = false;

volatile uint32_t lastGpsByteMillis = 0;
volatile uint32_t lastGpsSentenceMillis = 0;

double startLat = 0, startLon = 0, lastLat = 0, lastLon = 0;
double totalDist = 0.0;
unsigned long startTime = 0, lastLogTime = 0, lastDisplayTime = 0;

TaskHandle_t SensorTaskHandle;

void pollGpsInput() {
    while (Serial1.available() > 0) {
        gpsCommEverSeen = true;
        lastGpsByteMillis = millis();

        if (gps.encode(Serial1.read())) {
            lastGpsSentenceMillis = millis();
        }
    }
}

// --- RTOS SENSOR THREAD ---
void SensorTask(void *pvParameters) {
    (void) pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_INTERVAL_MS); 

    for (;;) {
        unsigned long currentMillis = millis();
        lastSensorTaskHeartbeat = currentMillis;
        uint8_t bank = activeBank;

        pollGpsInput();

        if (gps.location.isUpdated() && fixFound) {
            double distStep = gps.distanceBetween(gps.location.lat(), gps.location.lng(), lastLat, lastLon);
            if (distStep > DEADRECONING_DISTANCE_THRESHOLD) {
                totalDist += distStep;
                lastLat = gps.location.lat(); lastLon = gps.location.lng();
            }

            if (gpsCount[bank] < MAX_GPS_SAMPLES) {
                int idx = gpsCount[bank];
                gpsBuffer[bank][idx].timestamp = currentMillis;
                gpsBuffer[bank][idx].lat = gps.location.lat();
                gpsBuffer[bank][idx].lon = gps.location.lng();
                gpsBuffer[bank][idx].speed = gps.speed.kmph();
                gpsBuffer[bank][idx].distToStart = gps.distanceBetween(gps.location.lat(), gps.location.lng(), startLat, startLon);
                gpsBuffer[bank][idx].sats = gps.satellites.value();
                gpsBuffer[bank][idx].hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
                gpsBuffer[bank][idx].altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
                gpsBuffer[bank][idx].course = gps.course.isValid() ? gps.course.deg() : 0.0;
                gpsCount[bank]++;
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

            strokeDet.processNewData(currentMillis, a.acceleration.x, a.acceleration.y, a.acceleration.z, gps.speed.kmph());
            curveAnalyzer.update(currentMillis, strokeDet.getChosenSmooth(), strokeDet.getThreshold25());
            paceEstimator.update(currentMillis, strokeDet.getChosenSmooth(), gps.speed.kmph());
        } else if (!mpuOK) {
            // Rate limit error logging in loop
            if (currentMillis % 10000 < 20) logger.logEvent("ERROR", "Sensor thread running but MPU is disconnected");
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    delay(2000); // Allow monitor to connect

    // 1. Initialize Logger
    bool sdOK = logger.init();

    // 2. Initialize Display
    displayUI.init();
    
    // 3. Initialize MPU
    MyWire.begin();
    mpuOK = mpu.begin(MPU6050_I2CADDR_DEFAULT, &MyWire);
    if (mpuOK) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // 4. Initialize GPS
    Serial1.setPins(GPS_RX_PIN, GPS_TX_PIN);
    Serial1.begin(9600);

    bleController.beginBootMode();

    uint32_t bootTime = millis();
    uint32_t lastScreenUpdate = millis();

    displayUI.drawSetupScreen(sdOK, mpuOK, 0, "Waiting for GPS Fix...", 0);


    bool readyToRow = false;
    
    while (!readyToRow) {
        pollGpsInput();
        bleController.poll();
        
        // 1. Check GPS Fix
        if (!fixFound) {
            if (gps.location.isValid() && gps.date.year() > 2000 && 
                gps.satellites.isValid() && gps.satellites.value() >= 5 &&
                gps.hdop.isValid() && gps.hdop.hdop() < 2.0) {
                
                fixFound = true;
                startLat = gps.location.lat(); 
                startLon = gps.location.lng();
                lastLat = startLat; 
                lastLon = startLon;
                
                uint32_t currentMillis = millis();
                startTime = currentMillis;
                lastLogTime = currentMillis;
                lastDisplayTime = currentMillis;
                
                logger.createDataFile(gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute());
            }
        }

        uint32_t currentMillis = millis();
        uint32_t elapsedSec = (currentMillis - bootTime) / 1000;
        bool gpsUartAlive = gpsCommEverSeen || (currentMillis - bootTime < 5000);

        if (fixFound && !bleController.isBootActive()) {
            readyToRow = true;
            break;
        }
        // Update the normal boot screen every 10 seconds.
        if (currentMillis - lastScreenUpdate >= 10000) {
            lastScreenUpdate = currentMillis;
            
            String statusMsg;
            if (bleController.isConnected()) {
                statusMsg = "BLE connected";
            } else if (bleController.isBootActive()) {
                statusMsg = "BLE waiting";
            } else if (!gpsUartAlive) {
                statusMsg = "No GPS UART data - check wiring/baud";
            } else if (lastGpsSentenceMillis == 0) {
                statusMsg = "GPS data seen, waiting for fix";
            } else if (fixFound) {
                statusMsg = "GPS Locked. Starting...";
            } else {
                statusMsg = "Waiting for Fix...";
            }
            displayUI.drawSetupScreen(sdOK, mpuOK, gps.satellites.value(), statusMsg, elapsedSec);
        }
        
        delay(25); 
    }

    if (bleController.isBootActive()) {
        bleController.shutdown();
    }

    displayUI.drawBackground();

    // Start background sampling
    xTaskCreate(SensorTask, "Sensors", 2048, NULL, 2, &SensorTaskHandle);
}

// --- MAIN LOOP ---
void loop() {
    unsigned long currentMillis = millis();

    // --- 1. HEALTH CHECKS ---
    // Check if SensorTask has crashed/locked up on the MPU6050
    if (currentMillis - lastSensorTaskHeartbeat > 2000) {
        logger.logEvent("CRITICAL", "SensorTask has frozen! Possible I2C lockup.");
        lastSensorTaskHeartbeat = currentMillis; // reset to avoid log spam
    }

    // UI UPDATE INTERVAL
    if (currentMillis - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
        lastDisplayTime = currentMillis;

        uint32_t dispStart = millis();

        float newSpm = strokeDet.getCurrentSpm();

        if (spmCount[activeBank] < MAX_SPM_SAMPLES) {
            int idx = spmCount[activeBank];
            spmBuffer[activeBank][idx].timestamp = currentMillis;
            spmBuffer[activeBank][idx].spm = newSpm;
            spmCount[activeBank]++;
        } else {
            logger.logEvent("WARN", "SPM Buffer Overflowed!");
        }

        // Calculate 4s Speed avg
        float avgSpeed = paceEstimator.getSmoothedSpeedKmph();
        float totalTimeMin = (currentMillis - startTime) / 60000.0;
        // hour+2 adjusts for Stockholm local time
        displayUI.updateMainScreen(avgSpeed, totalDist, totalTimeMin, gps.satellites.value(), newSpm, gps.time.hour() + 2, gps.time.minute());
        
        uint32_t dispEnd = millis();
        if ((dispEnd - dispStart) > 1500) {
            char warnMsg[64];
            snprintf(warnMsg, sizeof(warnMsg), "WARN: Display update took %lu ms", (dispEnd - dispStart));
            logger.logEvent("WARN", warnMsg);
        }
      }

    // LOGGING INTERVAL
    if (currentMillis - lastLogTime >= LOG_INTERVAL_MS) {
        lastLogTime = currentMillis;

        if (imuCount[activeBank] >= MAX_IMU_SAMPLES) logger.logEvent("WARN", "IMU buffer full before flush! Data loss possible.");

        // Swap
        uint8_t processBank = activeBank; 
        activeBank = !activeBank;
        
        uint32_t sdStart = millis();

        // Flush
        logger.flushData(gpsBuffer[processBank], gpsCount[processBank], 
                         imuBuffer[processBank], imuCount[processBank], 
                         spmBuffer[processBank], spmCount[processBank]);

        uint32_t sdEnd = millis();
        
        // 1. Track System Memory Allocation (Standard ARM GCC method)
        struct mallinfo mi = mallinfo();
        uint32_t allocatedRAM = mi.uordblks;
        
        // 2. Track RTOS Task Stacks (Words remaining before crash)
        UBaseType_t loopStackRemaining = uxTaskGetStackHighWaterMark(NULL);
        UBaseType_t sensorStackRemaining = uxTaskGetStackHighWaterMark(SensorTaskHandle);
        
        // 3. Log all system health metrics
        char healthMsg[100];
        snprintf(healthMsg, sizeof(healthMsg), 
                 "SD_Time=%lu ms, AllocRAM=%lu B, LoopStack=%lu, SensorStack=%lu", 
                 (unsigned long)(sdEnd - sdStart), 
                 (unsigned long)allocatedRAM,
                 (unsigned long)loopStackRemaining,
                 (unsigned long)sensorStackRemaining);
                 
        logger.logEvent("HEALTH", healthMsg);

        // Reset
        imuCount[processBank] = 0;
        gpsCount[processBank] = 0;
        spmCount[processBank] = 0;
    }
  
    delay(50); 
}