#ifndef PACE_ESTIMATOR_H
#define PACE_ESTIMATOR_H

#include <Arduino.h>

class PaceEstimator {
private:
    // Tuning Parameters
    const float ALPHA_FUSION = 0.98f;      // Complementary filter weight (Trust IMU 98% for short-term)
    const float ALPHA_GRAVITY = 0.03f;     // Low-pass EMA for gravity (approx 0.5Hz cutoff at 100Hz)
    const float ALPHA_DISPLAY = 0.005f;    // Extremely slow EMA for display smoothing (approx 4s average)

    // State Variables
    float gravityY = 0.0f;
    float fusedVelocityMs = 0.0f;
    float displayVelocityMs = 0.0f;
    uint32_t lastUpdateMs = 0;
    bool isInitialized = false;

public:
    PaceEstimator();
    
    // Call this at 100Hz inside the SensorTask
    void update(uint32_t t_ms, float ay_m_s2, float gpsSpeedKmph);
    
    // Returns the smoothed, stable speed for the UI
    float getSmoothedSpeedKmph();
    
    // Returns the instantaneous surge speed (if you ever want to log it)
    float getInstantSpeedKmph();
};

#endif