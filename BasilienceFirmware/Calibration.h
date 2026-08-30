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
// Two-point calibration from final settled EMA voltage plateaus (see the
// targeted pH calibration update task):
//   pH 6.86 buffer -> 2562.40 mV
//   pH 4.01 buffer -> 2931.53 mV

constexpr float PH_SLOPE  = -0.00725505f;
constexpr float PH_OFFSET = 25.45033f;

#endif