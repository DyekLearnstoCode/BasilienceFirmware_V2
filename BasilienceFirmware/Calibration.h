#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>   // NAN - EC_CAL_2_VOLTAGE/EC_CAL_2_EC below are deliberately unset

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

// Retired: this was the single correction multiplier applied on top of a
// borrowed DFRobot TDS-sensor polynomial (see readEC()'s prior
// implementation). Real-hardware audit + calibration task found that
// polynomial was fit to a different probe/front-end and never matched this
// hardware - a known 12.88 mS/cm solution read only ~1.69 mS/cm through it,
// an error a single output multiplier cannot safely correct (the true error
// is not uniform across the range). Superseded by the EC_CAL_* anchor points
// below.
// constexpr float EC_FACTOR = 1.106f;

// ======================================================
// EC Calibration (anchor-point linear model)
// ======================================================
// Replaces the borrowed DFRobot TDS polynomial + EC_FACTOR above. See
// SensorManager::readEC() for the conversion itself:
//   - with only Point 1 confirmed (today): EC = EC_CAL_1_EC * (V / EC_CAL_1_VOLTAGE)
//     - a one-point PROPORTIONAL (through-origin) model. This is the most
//       that can be honestly justified from a single measured point - it
//       assumes the probe's voltage response passes through (0V, 0 mS/cm),
//       which is the standard assumption for this class of sensor absent a
//       second point, but is UNVALIDATED near 0 and unvalidated in the
//       1.2-2.0 mS/cm cultivation range specifically. Treat readings far
//       from EC_CAL_1_EC with appropriate caution until Point 2 is filled in.
//   - once Point 2 is also filled in: EC = EC_CAL_1_EC + (V - EC_CAL_1_VOLTAGE) * slope,
//     slope = (EC_CAL_2_EC - EC_CAL_1_EC) / (EC_CAL_2_VOLTAGE - EC_CAL_1_VOLTAGE)
//     - a genuine two-point linear fit, no longer assuming the origin.
//
// Point 1 - CONFIRMED on real hardware (EC calibration redesign task): probe
// dipped in a 12.88 mS/cm reference solution, EC module powered from 5V,
// ESP32 reading GPIO34. The voltage below (1.799V) was computed via the OLD
// raw-ADC * 3.3/4095 formula at the time it was measured (ADC~=2233), NOT
// yet through the new analogReadMilliVolts() path this task switches to -
// the two are expected to be close in this mid-range, but re-check this
// value against the new "[EC-CAL]" Serial diagnostic (readEC()) next time
// the probe is dipped in this same solution, and update it here if it
// differs meaningfully.
constexpr float EC_CAL_1_VOLTAGE = 1.799f;   // volts
constexpr float EC_CAL_1_EC      = 12.88f;   // mS/cm

// Point 2 - NOT YET CONFIRMED. Deliberately left unset (NAN): the task this
// section was added for explicitly does not fabricate a second point.
// Fill in both of these together once a second reference solution has
// actually been measured on this hardware - ideally one near the real
// 1.2-2.0 mS/cm cultivation range (EC_TARGET_MIN/EC_TARGET_MAX, Config.h),
// since that is where dosing decisions actually operate and a single
// high-concentration anchor cannot confirm accuracy there.
constexpr float EC_CAL_2_VOLTAGE = NAN;      // volts - unset
constexpr float EC_CAL_2_EC      = NAN;      // mS/cm - unset

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