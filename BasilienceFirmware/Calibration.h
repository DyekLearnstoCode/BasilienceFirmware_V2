#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

// ======================================================
// EC Sensor Calibration
// ======================================================
// Retired: EC now reads through ecSampler's MILLIVOLTS mode
// (analogReadMilliVolts(), see SensorManager::readEC()), the same
// ESP32-calibrated path pH already uses, instead of a raw ADC count run
// through this manual linear conversion. Kept only as a record of the ESP32
// hardware's own nominal specs (12-bit / 3.3V, matching analogReadResolution(12)
// / analogSetAttenuation(ADC_11db) in SensorManager::begin()) - nothing in
// the EC or pH path computes voltage from these anymore.
constexpr float ADC_REFERENCE = 3.3f;
constexpr int ADC_RESOLUTION = 4095;

// Retired: this was the single correction multiplier applied on top of the
// DFRobot TDS-sensor polynomial (see readEC()'s TDS-to-EC conversion below).
// Real-hardware audit + calibration task found that polynomial was fit to a
// different probe/front-end and never matched this hardware - a known
// 12.88 mS/cm solution read only ~1.69 mS/cm through it, an error a single
// output multiplier cannot safely correct (the true error is not uniform
// across the range, and only shows up that far outside the polynomial's
// drinking-water/TDS-meter design range - see the TDS-to-EC section below).
// Superseded for a while by the EC_CAL_* two-point anchor model (also now
// retired, see that section below); left off again here rather than
// reintroduced.
// constexpr float EC_FACTOR = 1.106f;

// ======================================================
// EC Calibration (anchor-point linear model) - RETIRED
// ======================================================
// Superseded again by the DFRobot TDS-to-EC conversion (below) - buffer-
// solution calibration was judged unreliable in practice (see that
// section's own reasoning) and this two-point line is no longer read by
// readEC(). Kept only as a record of what was measured; do not resurrect
// without re-validating against the same concerns that retired it twice.
//
// The model this fed was:
//   EC = EC_CAL_1_EC + (V - EC_CAL_1_VOLTAGE) * slope,
//   slope = (EC_CAL_2_EC - EC_CAL_1_EC) / (EC_CAL_2_VOLTAGE - EC_CAL_1_VOLTAGE)
//   - a two-point linear fit, anchored at a high-concentration point
//     (12.88 mS/cm) and one inside the real 1.2-2.0 mS/cm cultivation range
//     (1.4 mS/cm).
//
// Both points below were captured with the EC module wired through the
// sensor expansion board - the actual production signal path, not a probe
// straight to the ESP32's GPIO34. That distinction matters: the same
// solutions read very differently by path (e.g. the 1.4 mS/cm solution read
// ~1.73V direct to the ESP32, but consistently ~2.0-2.14V through the
// expansion board), confirmed by deliberately swapping wiring and watching
// the reading track the path, not the solution or supply.
//
// Point 1 - probe dipped in a 12.88 mS/cm reference solution, EC module
// powered from 5V, through the expansion board, ESP32 reading GPIO34.
// Landed on ~2.44V consistently - the stable anchor of the two. Re-captured
// at 2.443V in a later session, within normal session-to-session variation.
// constexpr float EC_CAL_1_VOLTAGE = 2.443f;   // volts
// constexpr float EC_CAL_1_EC      = 12.88f;   // mS/cm

// Point 2 - probe dipped in a 1.4 mS/cm reference solution, deliberately
// chosen near the real 1.2-2.0 mS/cm cultivation range (EC_TARGET_MIN/
// EC_TARGET_MAX, Config.h) rather than another high-concentration point.
// Several recaptures (1.734V direct-to-ESP32, then 2.007V/2.122V/2.142V
// through the expansion board) before the expansion-board wiring path was
// identified as the actual variable; re-captured again at 2.205V through
// the expansion board. These two points sit close together in voltage, so
// the fitted line was meaningfully steep - ordinary ADC noise showed up as
// a larger apparent swing in reported EC within the real cultivation range
// than a full-range calibration would give. That sensitivity to noise, on
// top of the buffer-solution readings themselves being judged unreliable,
// is why this model was retired in favor of the DFRobot TDS-to-EC
// conversion below.
// constexpr float EC_CAL_2_VOLTAGE = 2.205f;   // volts
// constexpr float EC_CAL_2_EC      = 1.40f;    // mS/cm

// ======================================================
// EC Calibration (DFRobot TDS-to-EC conversion) - ACTIVE
// ======================================================
// readEC() (SensorManager.cpp) now derives EC from the same DFRobot
// Gravity-TDS-sensor polynomial the EC_FACTOR-based approach used before the
// EC_CAL_* anchor model temporarily replaced it (see both retired sections
// above): TDS(ppm) is computed directly from voltage, then
// EC(mS/cm) = TDS / 500 - the vendor's own, unmodified TDS-to-EC ratio, with
// no extra multiplier layered on top (EC_FACTOR above stays retired/unused)
// and no per-device buffer-solution anchor. There is nothing to calibrate
// here by design: this trades the two-point model's better fit AT ITS
// anchor points for a conversion that isn't sensitive to how reliable those
// two buffer-solution readings were.

// ======================================================
// pH Calibration
// ======================================================
// Re-calibrated on the same SEN0161-V2 module (see git history for the
// prior 1544/2072 mV fit) after real-hardware testing found a consistent
// zero-offset drift: both buffers were reading ~0.25-0.4 pH too high
// (e.g. 4.35-4.4 instead of 4.00, 7.24-7.30 instead of 7.00) despite each
// being given a full 10+ minute settle and confirmed via a genuinely
// fresh, previously-unopened pH 4.0 buffer (ruling out buffer
// contamination). Same direction and similar magnitude in both buffers
// is the signature of electrode zero-point drift, not a slope/gain
// problem or instability - a normal thing pH electrodes do over time.
// Settled mV re-captured via tools/PhCalibrationDFRobot:
//   pH 7.00 buffer -> 1500 mV
//   pH 4.00 buffer -> 2038 mV

constexpr float PH_SLOPE  = -0.00557621f;
constexpr float PH_OFFSET = 15.36431f;

#endif