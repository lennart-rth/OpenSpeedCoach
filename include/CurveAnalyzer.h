#ifndef CURVE_ANALYZER_H
#define CURVE_ANALYZER_H

#include <Arduino.h>

struct HistPoint {
    uint32_t t;
    float val;
};

class CurveAnalyzer {
private:
    static const int HIST_SIZE = 1000; // 10 seconds of history at 100Hz
    static const uint32_t MIN_STROKE_TIME_MS = 900;

    HistPoint history[HIST_SIZE];
    int head = 0;
    int count = 0;

    // State Machine
    bool isBelowThreshold = false;
    float tempMinVal = 9999.0f;
    uint32_t tempMinTime = 0;
    uint32_t lastCatchTime = 0;

    // Stroke Data
    float recentStrokes[5][100];
    int strokeCount = 0;
    int strokeIdx = 0;

    // Output Data
    float averageStroke[100];
    float latestStroke[100];
    int latestFinishIdx = 0;
    float latestRatio = 0.0f;
    bool newStrokeAvailable = false;

    void extractAndNormalize(uint32_t tStart, uint32_t tEnd);
    void calculateMetricsAndAverage();

public:
    CurveAnalyzer();
    
    // Call this every 10ms with the smoothed data and threshold
    void update(uint32_t t_ms, float chosenSmooth, float threshold25);

    // Returns true if a new stroke was just finalized
    bool hasNewStroke();

    // Getters for the UI
    float* getLatestStroke();
    float* getAverageStroke();
    float getRhythmRatio();
    int getFinishIndex();
};

#endif