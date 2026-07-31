#ifndef CALIBRATION_H
#define CALIBRATION_H


// ======================================================
// EC Sensor Calibration
// ======================================================

constexpr float ADC_REFERENCE = 3.3f;
constexpr int ADC_RESOLUTION = 4095;

constexpr float EC_FACTOR = 1.106f;

// ======================================================
// pH Calibration
// ======================================================

constexpr float PH_SLOPE = -0.00571715f;
constexpr float PH_OFFSET = 21.535f;

#endif