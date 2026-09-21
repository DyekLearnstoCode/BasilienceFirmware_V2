#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>   // isfinite() in SensorManager::readEC()'s two-point/one-point model switch

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
// SensorManager::readEC() for the conversion itself. Both points below are
// now confirmed, so the active model is:
//   EC = EC_CAL_1_EC + (V - EC_CAL_1_VOLTAGE) * slope,
//   slope = (EC_CAL_2_EC - EC_CAL_1_EC) / (EC_CAL_2_VOLTAGE - EC_CAL_1_VOLTAGE)
//   - a genuine two-point linear fit, validated at both a high-concentration
//     anchor (12.88 mS/cm) and one inside the real 1.2-2.0 mS/cm cultivation
//     range (1.4 mS/cm), no longer assuming the origin.
// readEC() still falls back to a one-point PROPORTIONAL (through-origin) fit
// - EC = EC_CAL_1_EC * (V / EC_CAL_1_VOLTAGE) - if either EC_CAL_2_* constant
// is ever reset to NAN or the two points end up equal; that fallback is
// unvalidated near 0 and should be treated with appropriate caution if it's
// ever what's actually running.
//
// Both points below were captured with the EC module wired through the
// sensor expansion board - the actual production signal path, not a probe
// connected straight to the ESP32's GPIO34. That distinction matters: during
// this calibration task, the same solutions read very differently depending
// on which path was used (e.g. the 1.4 mS/cm solution read ~1.73V direct to
// the ESP32, but consistently ~2.0-2.14V through the expansion board), while
// direct-to-ESP32 readings stayed put across power sources and sessions.
// The expansion board itself is doing this, not probe contamination, not
// electrode drift, not the power source - confirmed by deliberately
// swapping between direct-to-ESP32 and through-the-expansion-board wiring
// and seeing the reading track the wiring path, not the solution or supply.
// Since the real device always reads EC through the expansion board, these
// two points are the correct ones to calibrate against, even though they
// don't match what a probe wired straight to the ESP32 would read.
//
// Point 1 - CONFIRMED on real hardware (EC calibration redesign task): probe
// dipped in a 12.88 mS/cm reference solution, EC module powered from 5V,
// through the expansion board, ESP32 reading GPIO34. Landed on ~2.44V every
// time it was captured this session, regardless of power source - the
// stable anchor of the two. Re-captured at 2.443V in a later session -
// updated below; still the same solution/wiring path, within normal
// session-to-session variation of the original ~2.44V reading.
constexpr float EC_CAL_1_VOLTAGE = 2.443f;   // volts
constexpr float EC_CAL_1_EC      = 12.88f;   // mS/cm

// Point 2 - CONFIRMED on real hardware (EC calibration redesign task): probe
// dipped in a 1.4 mS/cm reference solution, deliberately chosen near the
// real 1.2-2.0 mS/cm cultivation range (EC_TARGET_MIN/EC_TARGET_MAX,
// Config.h) rather than another high-concentration point, since that is
// where dosing decisions actually operate. Took several recaptures within
// the original session (1.734V direct-to-ESP32, then 2.007V/2.122V/2.142V
// through the expansion board) before the expansion-board wiring path was
// identified as the actual variable. Re-captured again in a later session at
// 2.205V through the expansion board - updated below; slightly widens the
// gap from Point 1 versus the original 2.142V capture, so the swing-per-mV
// concern noted below is somewhat less pronounced than originally measured,
// though still tighter than the very first 1.734V/12.88 pairing. Because
// these two points are still much closer together in voltage than that
// original pairing, the fitted line is meaningfully steeper than a
// full-range calibration would give - ordinary ADC noise will show up as a
// larger apparent swing in reported EC within the real cultivation range
// than it would with two more widely-separated anchors. Worth revisiting if
// that turns out to matter in practice (e.g. a lower-concentration second
// reference solution instead).
constexpr float EC_CAL_2_VOLTAGE = 2.205f;   // volts
constexpr float EC_CAL_2_EC      = 1.40f;    // mS/cm

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