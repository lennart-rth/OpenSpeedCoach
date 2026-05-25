#ifndef DISPLAY_H
#define DISPLAY_H

#include "DataTypes.h"
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>

class DisplayManager {
private:
    GxEPD2_BW<GxEPD2_270, GxEPD2_270::HEIGHT> display;
    
    float lastDispSpm = -1.0;
    int lastDispDist = -1;
    int lastDispMin = -1;
    int lastDispSplitMin = -1;
    int lastDispSplitSec = -1;
    int lastDispSats = -1;
    float lastDispRatio = -1.0;
    String lastDispStatus = "";

public:
    bool useClassicUI = true; 

    DisplayManager();
    void init();
    
    void drawSetupScreen(bool sdOK, bool mpuOK, int sats, String debugMsg, uint32_t searchTimeSec);
    void drawBluetoothScreen();
    void drawBackground();
    void updateMainScreen(float avgSpeed, float dist, float minutes, int sats, float currentSpm, int hour, int minute, float* avgCurve = nullptr, float ratio = 0.0f);
};

#endif