#ifndef LOGGER_H
#define LOGGER_H

#include "DataTypes.h"
#include <SdFat.h>

class Logger {
private:
    SdFat sd;
    File32 dataFile;
    File32 eventFile;
    bool sdOK = false;
    char logFileName[16];
    char eventFileName[16] = "events.log";

public:
    Logger();
    bool init();
    void createDataFile(int month, int day, int hour, int minute);
    void flushData(GpsData* gps, int gpsCnt, ImuData* imu, int imuCnt, SpmData* spm, int spmCnt);
    
    void logEvent(const char* level, const char* message, uint32_t timestamp = 0);

    SdFat& getSd() { return sd; }
};

#endif