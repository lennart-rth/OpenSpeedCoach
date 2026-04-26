#include "Logger.h"
#include <SPI.h>

Logger::Logger() {}

bool Logger::init() {
    SPI.setPins(PIN_MISO, PIN_SCK, PIN_MOSI);
    sdOK = sd.begin(SD_CS, SD_SCK_MHZ(12));
    if (sdOK) {
        logEvent("INFO", "SD Card Mount Successful");
    } else {
        Serial.println("CRITICAL ERROR: SD Card mount failed.");
    }
    return sdOK;
}

void Logger::logEvent(const char* level, const char* message, uint32_t timestamp) {
    // Print to serial console if attached
    Serial.print("["); Serial.print(level); Serial.print("] "); Serial.println(message);
    
    if (!sdOK) return;

    if (eventFile.open(eventFileName, O_RDWR | O_CREAT | O_APPEND)) {
        eventFile.print(millis()); eventFile.print(",");
        if (timestamp > 0) { eventFile.print(timestamp); } 
        else { eventFile.print(millis()); }
        eventFile.print(",");
        eventFile.print(level); eventFile.print(",");
        eventFile.println(message);
        eventFile.close();
    }
}

void Logger::createDataFile(int month, int day, int hour, int minute) {
    if (!sdOK) return;
    sprintf(logFileName, "%02d%02d_%02d%02d.csv", month, day, hour, minute);
    sprintf(eventFileName, "%02d%02d_%02d%02d_events.log", month, day, hour, minute);
    
    if (dataFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
        dataFile.println("Type,Millis,Lat,Lon,Speed_kmh,Dist_m,Sats,HDOP,Alt_m,Course_deg");
        dataFile.close();
        logEvent("INFO", "Created new data file");
    } else { 
        sdOK = false; 
        logEvent("ERROR", "Failed to create data file");
    }
}

void Logger::flushData(GpsData* gps, int gpsCnt, ImuData* imu, int imuCnt, SpmData* spm, int spmCnt) {
    if (!sdOK) return;

    if (dataFile.open(logFileName, O_RDWR | O_CREAT | O_APPEND)) {
        // 1. Write GPS Chunk
        for (int i = 0; i < gpsCnt; i++) {
            dataFile.print("GPS,");
            dataFile.print(gps[i].timestamp); dataFile.print(",");
            dataFile.print(gps[i].lat, 6); dataFile.print(",");
            dataFile.print(gps[i].lon, 6); dataFile.print(",");
            dataFile.print(gps[i].speed); dataFile.print(",");
            dataFile.print(gps[i].distToStart); dataFile.print(",");
            dataFile.print(gps[i].sats); dataFile.print(",");
            dataFile.print(gps[i].hdop); dataFile.print(",");
            dataFile.print(gps[i].altitude); dataFile.print(",");
            dataFile.println(gps[i].course);
        }

        // 2. Write IMU Chunk
        for (int i = 0; i < imuCnt; i++) {
            dataFile.print("IMU,");
            dataFile.print(imu[i].timestamp); dataFile.print(",");
            dataFile.print(imu[i].ax); dataFile.print(",");
            dataFile.print(imu[i].ay); dataFile.print(",");
            dataFile.print(imu[i].az); dataFile.print(",");
            dataFile.print(imu[i].gx); dataFile.print(",");
            dataFile.print(imu[i].gy); dataFile.print(",");
            dataFile.println(imu[i].gz);
        }

        // 3. Write SPM Chunk
        for (int i = 0; i < spmCnt; i++) {
            dataFile.print("SPM,");
            dataFile.print(spm[i].timestamp); dataFile.print(",");
            dataFile.println(spm[i].spm, 2);
        }
        
        dataFile.close();
        
        char flushMsg[64];
        snprintf(flushMsg, sizeof(flushMsg), "Flush OK. GPS:%d IMU:%d SPM:%d", gpsCnt, imuCnt, spmCnt);
        logEvent("INFO", flushMsg);
    } else {
        sdOK = false; 
        logEvent("ERROR", "File open failed during flush!");
    }
}

