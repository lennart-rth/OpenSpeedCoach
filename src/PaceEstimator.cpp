#include "PaceEstimator.h"

PaceEstimator::PaceEstimator() {}

void PaceEstimator::update(uint32_t t_ms, float ay_m_s2, float gpsSpeedKmph) {
    if (!isInitialized) {
        // Seed the filters with initial state to prevent a long ramp-up time
        gravityY = ay_m_s2; 
        fusedVelocityMs = gpsSpeedKmph / 3.6f;
        displayVelocityMs = fusedVelocityMs;
        lastUpdateMs = t_ms;
        isInitialized = true;
        return;
    }

    // 1. Calculate precise Delta Time (dt)
    float dt = (t_ms - lastUpdateMs) / 1000.0f;
    lastUpdateMs = t_ms;
    
    // Safety check: if thread paused for a long time, don't blow up the math
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f; 

    // 2. Isolate Gravity (Fast EMA)
    gravityY = (ALPHA_GRAVITY * ay_m_s2) + ((1.0f - ALPHA_GRAVITY) * gravityY);

    // 3. Extract Linear Acceleration
    // Assuming Y-axis points to the bow. If it points backwards, change to: gravityY - ay_m_s2
    float linearAccel = ay_m_s2 - gravityY; 

    // 4. Complementary Filter (Sensor Fusion)
    float gpsSpeedMs = gpsSpeedKmph / 3.6f;
    fusedVelocityMs = ALPHA_FUSION * (fusedVelocityMs + (linearAccel * dt)) + (1.0f - ALPHA_FUSION) * gpsSpeedMs;

    // 5. Display Smoothing (Slow EMA)
    // This replaces the "10 second GPS average" loop with a continuous math equivalent
    displayVelocityMs = (ALPHA_DISPLAY * fusedVelocityMs) + ((1.0f - ALPHA_DISPLAY) * displayVelocityMs);
}

float PaceEstimator::getSmoothedSpeedKmph() {
    // Prevent negative speeds from showing on the display when stopped
    if (displayVelocityMs < 0.1f) return 0.0f;
    return displayVelocityMs * 3.6f;
}

float PaceEstimator::getInstantSpeedKmph() {
    return fusedVelocityMs * 3.6f;
}