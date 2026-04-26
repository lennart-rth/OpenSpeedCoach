#include "CurveAnalyzer.h"

CurveAnalyzer::CurveAnalyzer() {
    for (int i = 0; i < 100; i++) {
        averageStroke[i] = 0.0f;
        latestStroke[i] = 0.0f;
    }
}

void CurveAnalyzer::update(uint32_t t_ms, float chosenSmooth, float threshold25) {
    // 1. Push to circular history buffer
    history[head].t = t_ms;
    history[head].val = chosenSmooth;
    head = (head + 1) % HIST_SIZE;
    if (count < HIST_SIZE) count++;

    // 2. Minimum Catch State Machine
    if (count > 20) {
        if (chosenSmooth < threshold25) {
            if (!isBelowThreshold) {
                isBelowThreshold = true;
                tempMinVal = chosenSmooth;
                tempMinTime = t_ms;
            } else {
                if (chosenSmooth < tempMinVal) {
                    tempMinVal = chosenSmooth;
                    tempMinTime = t_ms;
                }
            }
        } else if (isBelowThreshold) {
            isBelowThreshold = false;
            
            if (lastCatchTime == 0) {
                lastCatchTime = tempMinTime;
            } else if ((tempMinTime - lastCatchTime) > MIN_STROKE_TIME_MS) {
                // A complete stroke is captured!
                extractAndNormalize(lastCatchTime, tempMinTime);
                calculateMetricsAndAverage();
                newStrokeAvailable = true;
                lastCatchTime = tempMinTime;
            }
        }
    }
}

void CurveAnalyzer::extractAndNormalize(uint32_t tStart, uint32_t tEnd) {
    // Fast O(N) Forward Interpolation through the circular buffer
    int idx = (head - count + HIST_SIZE) % HIST_SIZE; // Oldest point
    
    for (int i = 0; i < 100; i++) {
        float target_t = tStart + i * (tEnd - tStart) / 99.0f;
        
        // Advance buffer index until we bracket the target_t
        while (true) {
            int next_idx = (idx + 1) % HIST_SIZE;
            if (next_idx == head || history[next_idx].t > target_t) {
                break;
            }
            idx = next_idx;
        }
        
        int next_idx = (idx + 1) % HIST_SIZE;
        if (next_idx == head) {
            latestStroke[i] = history[idx].val; // Reached end of buffer
        } else {
            float t0 = history[idx].t;
            float t1 = history[next_idx].t;
            float v0 = history[idx].val;
            float v1 = history[next_idx].val;
            
            if (t1 == t0) {
                latestStroke[i] = v0;
            } else {
                latestStroke[i] = v0 + ((target_t - t0) / (t1 - t0)) * (v1 - v0);
            }
        }
        
        // Save to recent strokes matrix
        recentStrokes[strokeIdx % 5][i] = latestStroke[i];
    }
    
    strokeIdx++;
    strokeCount = min(strokeCount + 1, 5);
}

void CurveAnalyzer::calculateMetricsAndAverage() {
    // 1. Calculate Average Stroke
    for (int i = 0; i < 100; i++) {
        float sum = 0;
        for (int j = 0; j < strokeCount; j++) {
            sum += recentStrokes[j][i];
        }
        averageStroke[i] = sum / strokeCount;
    }

    // 2. Detect Finish/Release
    int peakIdx = 0;
    float peakVal = -9999.0f;
    
    // Find peak in the first half (Drive)
    for (int i = 0; i < 50; i++) {
        if (latestStroke[i] > peakVal) {
            peakVal = latestStroke[i];
            peakIdx = i;
        }
    }

    latestFinishIdx = peakIdx;
    bool foundDip = false;
    
    // Look for the first local minimum (the release check)
    for (int i = peakIdx; i < 90; i++) {
        if (latestStroke[i] < latestStroke[i+1]) {
            latestFinishIdx = i;
            foundDip = true;
            break;
        }
    }
    
    // Fallback: If no clear dip, look for where the curve flattens out
    if (!foundDip) {
        for (int i = peakIdx; i < 90; i++) {
            if ((latestStroke[i+1] - latestStroke[i]) > -0.02f) {
                latestFinishIdx = i;
                break;
            }
        }
    }

    // 3. Calculate Rhythm Ratio
    float drivePct = latestFinishIdx;
    float recPct = 100.0f - latestFinishIdx;
    
    if (recPct > 0) {
        latestRatio = drivePct / recPct;
    } else {
        latestRatio = 0.0f;
    }
}

bool CurveAnalyzer::hasNewStroke() {
    if (newStrokeAvailable) {
        newStrokeAvailable = false;
        return true;
    }
    return false;
}

float* CurveAnalyzer::getLatestStroke() { return latestStroke; }
float* CurveAnalyzer::getAverageStroke() { return averageStroke; }
float CurveAnalyzer::getRhythmRatio() { return latestRatio; }
int CurveAnalyzer::getFinishIndex() { return latestFinishIdx; }