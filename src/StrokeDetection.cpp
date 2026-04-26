#include "StrokeDetection.h"

StrokeDetection::StrokeDetection() {}

float StrokeDetection::calculateVariance(float* arr, int size) {
    float sum = 0, sqDiffSum = 0;
    for (int i = 0; i < size; i++) sum += arr[i];
    float mean = sum / size;
    for (int i = 0; i < size; i++) {
        float diff = arr[i] - mean;
        sqDiffSum += diff * diff;
    }
    return sqDiffSum / size;
}

void StrokeDetection::finalizeStroke(uint32_t strokeTime) {
    if (lastFinalizedStrokeTime != 0) {
        float dt = (strokeTime - lastFinalizedStrokeTime) / 1000.0f;
        float rawSpm = 60.0f / dt;
        
        if (rawSpm >= 10.0f && rawSpm <= 65.0f) {
            spmHistory[spmHistIdx % 3] = rawSpm;
            spmHistIdx++;
            spmHistCount = min(spmHistCount + 1, 3);

            float temp[3];
            for (int i = 0; i < spmHistCount; i++) temp[i] = spmHistory[i];
            
            // Simple Sort
            for (int i = 0; i < spmHistCount - 1; i++) {
                for (int j = i + 1; j < spmHistCount; j++) {
                    if (temp[i] > temp[j]) { float t = temp[i]; temp[i] = temp[j]; temp[j] = t; }
                }
            }
            float medianSpm = temp[spmHistCount / 2];

            // 2. Fast EMA (80% New Pace, 20% Old Pace)
            if (currentEma == 0) currentEma = medianSpm;
            else currentEma = (medianSpm * 0.8f) + (currentEma * 0.2f);
        }
    }
    lastFinalizedStrokeTime = strokeTime;
}

void StrokeDetection::processNewData(uint32_t t_ms, float ax, float ay, float az, float speedKmph) {
    if (speedKmph < 0.5f) {
        if (isMoving) {
            currentEma = 0;
            hasPendingStroke = false;
            lastFinalizedStrokeTime = 0;
            spmHistCount = 0;
        }
        isMoving = false;
        return;
    }
    isMoving = true;

    float xs = filterX.process(ax);
    float ys = filterY.process(ay);
    float zs = filterZ.process(az);

    xBuf[domIdx] = xs; yBuf[domIdx] = ys; zBuf[domIdx] = zs;
    int currentCount = domIdx + 1; 
    domIdx = (domIdx + 1) % DOM_BUFFER_SIZE;

    if (t_ms - lastDomEvalTime >= 1000 && currentCount >= 100) {
        int evalSize = min(DOM_BUFFER_SIZE, currentCount);
        float varX = calculateVariance(xBuf, evalSize);
        float varY = calculateVariance(yBuf, evalSize);
        float varZ = calculateVariance(zBuf, evalSize);
        if (varX > varY && varX > varZ) currentDomAxis = 1;
        else if (varY > varX && varY > varZ) currentDomAxis = 2;
        else currentDomAxis = 3;
        lastDomEvalTime = t_ms;
    }

    float chosenRaw = (currentDomAxis == 1) ? xs : ((currentDomAxis == 2) ? ys : zs);
    float chosenSmooth = filterChosen.process(chosenRaw);

    chosenBuf[chosenIdx] = chosenSmooth;
    int minMaxCount = min(MIN_MAX_BUFFER_SIZE, (chosenIdx + 1 > MIN_MAX_BUFFER_SIZE) ? MIN_MAX_BUFFER_SIZE : chosenIdx + 1);
    chosenIdx = (chosenIdx + 1) % MIN_MAX_BUFFER_SIZE;

    float rMin = chosenBuf[0], rMax = chosenBuf[0];
    for (int i = 1; i < minMaxCount; i++) {
        if (chosenBuf[i] < rMin) rMin = chosenBuf[i];
        if (chosenBuf[i] > rMax) rMax = chosenBuf[i];
    }
    float threshold25 = rMin + 0.25f * (rMax - rMin);

    if (minMaxCount > 20) { 
        if (prevChosenSmooth < prevThreshold && chosenSmooth >= threshold25) {
            if (!hasPendingStroke) {
                pendingStrokeTime = t_ms; pendingStrokeVal = chosenSmooth; hasPendingStroke = true;
            } else {
                if ((t_ms - pendingStrokeTime) <= MIN_STROKE_TIME_MS) {
                    pendingStrokeTime = t_ms; pendingStrokeVal = chosenSmooth;
                } else {
                    finalizeStroke(pendingStrokeTime);
                    pendingStrokeTime = t_ms; pendingStrokeVal = chosenSmooth;
                }
            }
        } else if (hasPendingStroke && (t_ms - pendingStrokeTime) > MIN_STROKE_TIME_MS) {
            finalizeStroke(pendingStrokeTime);
            hasPendingStroke = false;
        }
    }
    prevChosenSmooth = chosenSmooth;
    prevThreshold = threshold25;
}

float StrokeDetection::getCurrentSpm() { return currentEma; }
float StrokeDetection::getChosenSmooth() { return prevChosenSmooth; }
float StrokeDetection::getThreshold25() { return prevThreshold; }