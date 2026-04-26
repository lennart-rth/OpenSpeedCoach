#ifndef STROKE_DETECTION_H
#define STROKE_DETECTION_H

#include <Arduino.h>

struct EmaFilter {
    float alpha;
    float y;
    bool initialized;

    EmaFilter(float a = 0.3f) {
        alpha = a;
        y = 0.0f;
        initialized = false;
    }

    float process(float x) {
        if (!initialized) {
            y = x;
            initialized = true;
        } else {
            y = (alpha * x) + ((1.0f - alpha) * y);
        }
        return y;
    }
};

class StrokeDetection {
private:
    static const int DOM_BUFFER_SIZE = 2000; 
    static const int MIN_MAX_BUFFER_SIZE = 1000; 
    static const uint32_t MIN_STROKE_TIME_MS = 900; 
    
    EmaFilter filterX{0.3f}, filterY{0.3f}, filterZ{0.3f};
    EmaFilter filterChosen{0.2f};

    float xBuf[DOM_BUFFER_SIZE], yBuf[DOM_BUFFER_SIZE], zBuf[DOM_BUFFER_SIZE];
    int domIdx = 0;
    int currentDomAxis = 1; 
    uint32_t lastDomEvalTime = 0;

    float chosenBuf[MIN_MAX_BUFFER_SIZE];
    int chosenIdx = 0;
    
    uint32_t pendingStrokeTime = 0;
    float pendingStrokeVal = 0;
    bool hasPendingStroke = false;
    uint32_t lastFinalizedStrokeTime = 0;
    
    float prevChosenSmooth = 0;
    float prevThreshold = 0;

    float spmHistory[3] = {0, 0, 0};
    int spmHistIdx = 0;
    int spmHistCount = 0;
    float currentEma = 0;
    bool isMoving = false;

    float calculateVariance(float* arr, int size);
    void finalizeStroke(uint32_t strokeTime);

public:
    StrokeDetection();
    void processNewData(uint32_t t_ms, float ax, float ay, float az, float speedKmph);
    float getCurrentSpm();
    float getChosenSmooth();
    float getThreshold25();
};


#endif