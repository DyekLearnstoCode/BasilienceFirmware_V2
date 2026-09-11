#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "Version.h"

#define OPERATION_TIMEOUT_MS 300000UL

// ======================================================
// Config/Settings Schema Migration
// ======================================================
// Bumped whenever a compiled default changes in a way that could disagree
// with a value an already-deployed device has already persisted (NVS and/or
// Firebase) from a previous firmware version - a stored value is
// indistinguishable at the value level alone from a genuine admin choice,
// so reconciling it must be a one-time, explicitly-versioned migration, not
// a "does it equal the old default" heuristic that could later clobber a
// deliberate admin setting that happens to match. See
// FirebaseManager::loadPersistedSettings()/readSettings() for the actual
// migration steps this version gates (NVS key "cfgVersion").
//   v1 (this version): air-temperature monitored maximum corrected 28C ->
//     32C (systemState.maxAirTemp/TARGET_MAX_AIR_TEMP), automatic
//     root-blower fogging speed corrected 30% -> 65%
//     (systemState.blowerSpeedPercent/BLOWER_SPEED_DEFAULT_PERCENT).
constexpr uint8_t CONFIG_SCHEMA_VERSION = 1;

// Bounded local fallback: how long Fogger/Blower resume waits after a local
// pH/EC correction completes for RTDB COMPLETED publication before releasing
// from local safe state anyway, so plant control never depends on cloud
// availability.
constexpr unsigned long CHEMISTRY_FOGGING_HOLD_TIMEOUT_MS = 30000UL;

// Minimum spacing between completed DS18B20 conversions. The sensor read
// itself is a blocking OneWire transaction, so it must not run every loop
// iteration - both to stop it from dominating loop() timing and to reduce
// how often it can collide with other blocking work (e.g. Firebase calls).
constexpr unsigned long WATER_TEMP_READ_INTERVAL_MS = 5000UL;

// Minimum spacing between HC-SR04 trigger pulses. Without this, readWaterLevel()
// re-triggers on literally every loop iteration - far faster than the sensor's
// own echo/reverberation settling time - which is a common cause of spurious
// pulseIn() timeouts unrelated to the sensor or wiring actually failing.
constexpr unsigned long WATER_LEVEL_READ_INTERVAL_MS = 5000UL;

// Minimum spacing between DHT22 samples. Confirmed marginal at the previous
// 2000ms value (DHT22 intermittent-communication audit): 2000ms is exactly
// the DHT22 datasheet's stated minimum sampling period AND exactly the DHT
// library's own internal MIN_INTERVAL floor (DHT.cpp) - i.e. readDHT() was
// polling right at the sensor's hard floor with zero margin. 2500ms keeps
// the same debounce/EMA/threshold behavior (only the cadence changes) while
// giving genuine headroom above that floor. Without a spacing gate at all,
// readDHT() would re-sample on literally every loop iteration - far faster
// than the sensor can actually answer - which is why humidity/air
// temperature would intermittently blank out and reappear even though the
// sensor itself never lost contact.
constexpr unsigned long DHT_READ_INTERVAL_MS = 5000UL;

// DHT22 physical measurement range (datasheet: -40..80C, 0..100% RH) - pure
// SENSOR VALIDITY, never an agronomic/automation threshold (28C or 10C are
// both physically valid readings, whatever the cultivation target is). Used
// only to reject an impossible/corrupted raw sample (e.g. 587.96C) BEFORE
// it can reach dhtTemperatureFiltered/dhtHumidityFiltered - see readDHT()'s
// own comment for the confirmed bug this fixes.
constexpr float DHT22_MIN_TEMP_C = -40.0f;
constexpr float DHT22_MAX_TEMP_C = 80.0f;
constexpr float DHT22_MIN_HUMIDITY_PCT = 0.0f;
constexpr float DHT22_MAX_HUMIDITY_PCT = 100.0f;

// Throttle for readDHT()'s [DHT-RAW] diagnostic's VALID case only - an
// invalid raw sample is always printed immediately (already naturally rate-
// limited to once per DHT_READ_INTERVAL_MS, and each one is the evidence
// this diagnostic exists for).
constexpr unsigned long DHT_RAW_DIAGNOSTIC_INTERVAL_MS = 5000UL;

// Exponential-smoothing weight given to each freshly accepted raw DHT22/
// DS18B20 reading (0..1 - higher tracks the raw sensor faster but smooths
// less, lower smooths more but lags a genuine change more). 0.3 converges
// to ~90% of a real step change within about 6 accepted samples
// (DHT ~15s at DHT_READ_INTERVAL_MS, DS18B20 ~6s at
// WATER_TEMP_READ_INTERVAL_MS) while suppressing normal per-sample sensor
// noise - see readDHT()/readWaterTemperature()'s own comments.
constexpr float DHT_SMOOTHING_ALPHA = 0.3f;
constexpr float WATER_TEMP_SMOOTHING_ALPHA = 0.3f;

// How long a unit that booted into a PERSISTED mock source waits for a fresh
// mock payload before giving up and reverting to physical sensors.
//
// Mock readings are deliberately never persisted, so a unit that reboots with
// mock mode still stored has no values to work from. Without this bound it
// would sit idle indefinitely whenever the cloud never came back. This applies
// ONLY to that boot-restored-without-payload window - it is not a general mock
// inactivity timer, and a mock session enabled explicitly after boot is never
// subject to it.
constexpr unsigned long MOCK_BOOT_PAYLOAD_TIMEOUT = 120000UL; // 2 minutes

// Developer Dynamic Mock Readings. Firmware owns this cadence so automation,
// alerts, Firebase publication, and Android Monitoring all consume the same
// effective values. Each update is a small bounded random-walk step around
// the developer-supplied base, never a fresh full-envelope resample.
constexpr unsigned long MOCK_DYNAMIC_UPDATE_INTERVAL_MS = 10000UL;
constexpr float MOCK_DYNAMIC_PH_ENVELOPE = 0.05f;
constexpr float MOCK_DYNAMIC_PH_STEP = 0.015f;
constexpr float MOCK_DYNAMIC_EC_ENVELOPE = 0.05f;
constexpr float MOCK_DYNAMIC_EC_STEP = 0.015f;
constexpr float MOCK_DYNAMIC_AIR_TEMP_ENVELOPE = 0.5f;
constexpr float MOCK_DYNAMIC_AIR_TEMP_STEP = 0.15f;
constexpr float MOCK_DYNAMIC_HUMIDITY_ENVELOPE = 1.5f;
constexpr float MOCK_DYNAMIC_HUMIDITY_STEP = 0.4f;
constexpr float MOCK_DYNAMIC_WATER_TEMP_ENVELOPE = 0.3f;
constexpr float MOCK_DYNAMIC_WATER_TEMP_STEP = 0.1f;
constexpr float MOCK_DYNAMIC_WATER_LEVEL_ENVELOPE = 0.75f;
constexpr float MOCK_DYNAMIC_WATER_LEVEL_STEP = 0.25f;

// Shared short debounce threshold used to tell a transient one-tick sensor
// hiccup (OneWire/ADC noise, a blocking call landing at the wrong moment)
// apart from a genuinely failed/disconnected sensor. Applied consistently to
// water-temperature confirmation, sensorFault, and the pH/EC/water-temp
// safety validity checks that can abort an active operation.
constexpr uint8_t SENSOR_TRANSIENT_FAILURE_THRESHOLD = 3;

// ======================================================
// WiFi
// ======================================================

//#define WIFI_SSID "EVITH WIFI"
//#define WIFI_PASSWORD "Ronald123"

#define WIFI_SSID "Jake"
#define WIFI_PASSWORD "walongone"

// ======================================================
// Firebase
// ======================================================

#define API_KEY "AIzaSyDaJ7F8tAREnCo7zrrY_sJ6SgfNuYQtra0"

#define DATABASE_URL \
    "https://basilience-database-default-rtdb.asia-southeast1.firebasedatabase.app"

// ======================================================
// Secure Device Auth (per-device bootstrap + refresh-token identity)
// ======================================================

// TEMPORARY migration flag. false (default) = legacy anonymous Firebase auth
// remains available as a fallback whenever this device has no bootstrap
// secret provisioned yet - required so already-fielded devices (including
// the current test unit, which has not had a secret injected yet) are not
// locked out the moment this firmware ships. Once every fielded device has
// been confirmed to hold a secret and successfully bootstrap, set this to
// true (forbidding the anonymous fallback) BEFORE restrictive RTDB rules are
// ever deployed - see the Secure Device Auth report's deployment checklist.
// Never silently left false in a "final" build; its state must always be a
// deliberate, reported decision.
constexpr bool SECURE_DEVICE_AUTH_REQUIRED = false;

// Cloud Function HTTPS endpoint that verifies a device's bootstrap secret and
// mints a Firebase custom token (uid = deviceId). PROPOSED path/region,
// matching this project's existing asia-southeast1 Firebase region - verify
// against the actual deployed function URL before physical use; not yet
// deployed as of this task.
#define BOOTSTRAP_ENDPOINT_URL "https://asia-southeast1-basilience-database.cloudfunctions.net/deviceAuthBootstrap"

// Google Trust Services GTS Root R1 - fetched directly from Google's own
// published trust store (https://pki.goog/repo/certs/gtsr1.pem), not
// transcribed from memory. Cloud Functions/Cloud Run HTTPS endpoints chain up
// to a Google Trust Services root; this is the long-lived root itself (valid
// to 2036), not a short-lived leaf certificate, so it should not need
// frequent rotation - but reconfirm against pki.goog if the bootstrap
// endpoint ever fails TLS validation unexpectedly.
constexpr const char* BOOTSTRAP_CA_CERT = R"CERT(
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo
27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w
Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw
TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl
qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH
szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8
Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk
MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92
wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p
aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN
VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb
C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe
QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy
h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4
7HLwEXWdyzRSjeZ2axfG34arJ45JK3VmgRAhpuo+9K4l/3wV3s6MJT/KYnAK9y8J
ZgfIPxz88NtFMN9iiMG1D53Dn0reWVlHxYciNuaCp+0KueIHoI17eko8cdLiA6Ef
MgfdG+RCzgwARWGAtQsgWSl4vflVy2PFPEz0tv/bal8xa5meLMFrUKTX5hgUvYU/
Z6tGn6D/Qqc6f1zLXbBwHSs09dR2CQzreExZBfMzQsNhFRAbd03OIozUhfJFfbdT
6u9AWpQKXCBfTkBdYiJ23//OYb2MI3jSNwLgjt7RETeJ9r/tSQdirpLsQBqvFAnZ
0E6yove+7u7Y/9waLd64NnHi/Hm3lCXRSHNboTXns5lndcEZOitHTtNCjv0xyBZm
2tIMPNuzjsmhDYAPexZ3FL//2wmUspO8IFgV6dtxQ/PeEMMA3KgqlbbC1j+Qa3bb
bP6MvPJwNQzcmRk13NfIRmPVNnGuV/u3gm3c
-----END CERTIFICATE-----
)CERT";

// ======================================================
// SSR Outputs
// ======================================================

constexpr uint8_t FOGGER_PIN = 26;
constexpr uint8_t GROW_LIGHT_PIN = 25;

// ======================================================
// MOSFET Outputs
// ======================================================

constexpr uint8_t BLOWER_PIN = 27;
constexpr uint8_t SOLENOID_PIN = 15;

// ======================================================
// Peristaltic Pumps
// ======================================================

constexpr uint8_t GROW_PUMP_PIN = 32;
constexpr uint8_t BLOOM_PUMP_PIN = 33;

constexpr uint8_t PH_UP_PUMP_PIN = 14;
constexpr uint8_t PH_DOWN_PUMP_PIN = 12;

// ======================================================
// Temperature
// ======================================================

constexpr uint8_t CANOPY_FAN_PIN = 17;
constexpr uint8_t PELTIER_PIN = 16;
constexpr uint8_t CIRCULATION_PUMP_PIN = 13;

// ======================================================
// Sensor Inputs
// ======================================================

constexpr uint8_t DHT_PIN = 4;
#define DHTTYPE DHT22

constexpr uint8_t WATER_TEMP_PIN = 5;

constexpr uint8_t EC_PIN = 34;

constexpr uint8_t PH_SENSOR_PIN = 35;

constexpr uint8_t TRIG_PIN = 18;
constexpr uint8_t ECHO_PIN = 19;

constexpr uint8_t RTC_SDA_PIN = 21;
constexpr uint8_t RTC_SCL_PIN = 22;

// ======================================================
// GSM / SIM800L
// ======================================================
// Confirmed non-conflicting production wiring - does not overlap with any
// sensor, actuator, I2C, or UART0 (USB/debug) pin above. Unchanged from the
// previous A76XX-family module - same pins, same UART framing (8N1).
constexpr uint8_t GSM_RX_PIN = 36;  // ESP32 RX <- SIM800L TXD
constexpr uint8_t GSM_TX_PIN = 23;  // ESP32 TX -> SIM800L RXD

// Bench-confirmed baud for the physically wired SIM800L V2 (blue board,
// SIM800 R13.08 firmware) on Smart/SMART Gold (PH) - see the GSM physical
// validation report. An earlier revision of this firmware probed a list of
// candidate bauds because the previously-installed LTE module's rate wasn't
// knowable in advance; this specific module only ever answers at 9600, so
// GsmManager now opens the UART here directly instead of cycling candidates.
constexpr unsigned long GSM_BAUD_RATE = 9600UL;

// ======================================================
// Sensor Thresholds
// ======================================================

constexpr float MIN_HUMIDITY = 60.0f;
constexpr float MAX_HUMIDITY = 80.0f;

constexpr float MIN_PH = 5.5f;
constexpr float MAX_PH = 6.5f;
constexpr float PH_TARGET_MIN = 5.8f;
constexpr float PH_TARGET_MAX = 6.3f;

constexpr float MIN_EC = 1.2f;
constexpr float MAX_EC = 2.0f;
constexpr float EC_TARGET_MIN = 1.4f;
constexpr float EC_TARGET_MAX = 1.8f;

// ======================================================
// pH/EC Last-Stable-Value Filter
// ======================================================
// Second-stage stability gate over readPH()/readEC()'s already ~1s-averaged
// output - see SensorManager::updateStabilityWindow() and
// applyEffectiveSensors(). A sliding window of STABILITY_SAMPLE_WINDOW
// samples, taken roughly STABILITY_SAMPLE_INTERVAL_MS apart, must all agree
// within the tolerance below before sensors.ph/sensors.ec (the ONE dataset
// AutomationManager/AlertManager/SafetyManager/Firebase publication all
// consume - no separate raw path feeds any of them) accept a new value; an
// unstable window keeps the previous accepted value instead of
// publishing/acting on the fluctuation. Calibration (PH_SLOPE/PH_OFFSET,
// EC_FACTOR - Calibration.h) is untouched by this filter; it only decides
// when an already-calibrated reading is trustworthy enough to act on.
//
// Sampling cadence: readPH()/readEC() recompute their rolling average every
// loop() tick, but the underlying ring buffer only advances by one raw ADC
// sample every PH_SAMPLE_INTERVAL/EC_SAMPLE_INTERVAL (20ms) - consecutive
// loop-tick reads of that average are therefore heavily autocorrelated
// (49-50 of 51/61 underlying samples unchanged between them) and would make
// "10 agreeing samples" trivially true even mid-excursion. 1000ms is
// approximately one full turnover of that underlying rolling average (51-61
// samples * 20ms =~1.0-1.2s), so each stability-window sample is a
// genuinely fresh observation rather than a near-duplicate of the last.
constexpr uint8_t STABILITY_SAMPLE_WINDOW = 10;
constexpr unsigned long STABILITY_SAMPLE_INTERVAL_MS = 1000UL;

// Starting point per the task spec - tight enough to still reject genuine
// probe noise, loose enough that 10 samples (~10s) reliably converge once
// the reading has actually settled. Re-tune only against observed
// near-threshold noise amplitude, never as a stand-in for fixing a noisy
// connection.
constexpr float PH_STABILITY_TOLERANCE = 0.05f;

// EC's anchor-point calibration (Calibration.h) maps the 1.2-2.0 mS/cm
// cultivation range to only ~168-280mV at the probe - the same 0.05 mS/cm
// tolerance used for pH would allow just ~7mV of jitter there, tighter than
// the ~10mV of ADC movement already observed on this hardware's flatter pH
// channel (see SensorManager's own real-hardware note). Widened to keep the
// window able to close at real operating voltage; this is an estimate, not
// a measured value - re-tune against the EC-CAL diagnostic's actual jitter
// once the probe is dipped at cultivation-range EC.
constexpr float EC_STABILITY_TOLERANCE = 0.1f;

// ======================================================
// pH Temporal Step Filter
// ======================================================
// Second stage, applied BEFORE the pH stability window above, mirroring the
// HC-SR04 water-depth step filter's design (see readWaterLevel()'s own
// comment and WATER_LEVEL_STEP_* below). A physically-valid pH candidate
// (0.0-14.0, already guarded elsewhere) can still transiently jump (e.g.
// 6.3 -> 7.3 -> 6.2) without representing a real pH change - reservoir
// electrical noise, not a genuine dose-worthy event. A candidate within
// PH_TELEMETRY_DEADBAND of the current trusted anchor (lastAcceptedPhCandidate)
// is treated as noise and never moves the anchor; a candidate beyond the
// deadband is held pending until PH_STEP_CONFIRM_COUNT consecutive
// candidates mutually agree within PH_STEP_CONFIRM_TOLERANCE - only then
// does the new level replace the anchor and get offered to the existing
// 10-sample stability window, which still independently decides whether
// that trusted stream itself is stable enough to become authoritative.
// This filter does not replace that window; it only decides what
// candidate stream the window ever sees.
//
// pH telemetry ratcheting fix: the anchor used to also accept any
// candidate within PH_STEP_ACCEPT_DELTA (0.15) of itself IMMEDIATELY, no
// confirmation streak required. Chained across many 300ms ticks, a slow
// noisy drift could walk the anchor an arbitrary distance in one
// direction (6.50 -> 6.62 -> 6.74 -> 6.86 -> ...) even though no single
// step exceeded 0.15, since each accepted step simply became the new
// anchor the next comparison was measured against. PH_TELEMETRY_DEADBAND
// replaces that immediate-accept path: the anchor now only ever moves
// after PH_STEP_CONFIRM_COUNT candidates confirm a genuinely new level
// relative to the SAME still-unmoved anchor, so small correlated steps in
// one direction can no longer accumulate into an unconfirmed drift.
constexpr float PH_TELEMETRY_DEADBAND = 0.05f;

// Superseded by PH_TELEMETRY_DEADBAND above for the anchor's own
// accept-vs-hold decision (see the ratcheting fix comment) - left defined,
// unused by SensorManager, only in case a future tuning pass wants the
// distinction back.
constexpr float PH_STEP_ACCEPT_DELTA = 0.15f;
constexpr float PH_STEP_CONFIRM_TOLERANCE = 0.05f;
constexpr uint8_t PH_STEP_CONFIRM_COUNT = 3;

// Quick-response refinement: the step filter's own evaluation cadence,
// deliberately separate from STABILITY_SAMPLE_INTERVAL_MS (1000ms) - the
// automation-trust stability window below still samples at that slower,
// stricter cadence unchanged. This is a dedicated, faster cadence purely
// for how often the TEMPORAL FILTER itself pulls a fresh candidate from
// the continuously-updating 51-sample median - fast enough that 3
// confirmations (PH_STEP_CONFIRM_COUNT) complete in ~3x this interval
// (~0.75-1.2s at 300ms), slow enough that consecutive evaluations are
// still genuinely distinct observations rather than re-evaluating one
// barely-changed rolling median value from adjacent loop() ticks (each
// individual raw ADC sample only refreshes every PH_SAMPLE_INTERVAL=20ms,
// so 250-400ms already spans several fresh raw samples sliding through
// the median).
constexpr unsigned long PH_STEP_SAMPLE_INTERVAL_MS = 300UL;

// ======================================================
// pH/EC Hardware-Fault Detection (rail-proximity, conservative)
// ======================================================
// Distinct from the domain/step/stability validation above, which already
// catches an implausible-but-mid-range candidate (e.g. pH 24.1 from a
// floating-but-not-rail-stuck input) - this layer instead inspects the RAW
// millivolt signal (physicalSensors.phMilliVolts/ecRaw) for evidence of a
// genuine electrical fault: a signal pinned at/near the ADC's own physical
// rail, which no chemistry reading through this transmitter/module can ever
// legitimately produce. Deliberately NOT a "normal cultivation voltage
// range" - see this task's own design principle: a chemically unusual
// solution (e.g. a very high or very low but real EC/pH) must never be
// mistaken for a disconnected probe. These margins are anchored to the
// hardware's own physical/documented limits, not to any agronomic range.
//
// Precedent this margin is chosen against: the pH module's own prior
// documented failure (see basilience_ph_calibration.md, 2026-09-05..08) was
// a dead amplifier board found - via multimeter, directly on the module's
// signal pin - pinned at a stable 5V regardless of the probe. On this
// 3.3V-max ESP32 ADC input, a fault like that reads as pinned at/near the
// ADC's own high rail, exactly what PH_FAULT_RAIL_HIGH_MV/EC_FAULT_RAIL_HIGH_MV
// below are built to catch. A low-rail fault (grounded/shorted/unpowered
// signal line) is the symmetric case at the bottom of the range.
//
// pH: the DFRobot Gravity Analog pH Meter V2 (SEN0161-V2) transmitter's own
// documented output spec is 0-3.0V, so PH_FAULT_RAIL_HIGH_MV sits just under
// that spec's own ceiling - a reading AT or ABOVE the transmitter's own
// stated maximum is not a value it should ever produce from a probe, on any
// solution, within its designed operating range. Cross-checked against this
// unit's own calibration (Calibration.h: pH 7.00 ~= 1500mV, pH 4.00 ~=
// 2038mV): even the full 0-14 pH domain extrapolates to roughly 245-2755mV,
// comfortably inside both rail margins with real margin to spare.
constexpr int PH_FAULT_RAIL_LOW_MV = 50;
constexpr int PH_FAULT_RAIL_HIGH_MV = 2950;

// EC: the module's exact model/transmitter spec is NOT known (see the
// sensor-inventory audit - it is only known to be an analog TDS-style probe,
// 5V-powered, read via GPIO34's analogReadMilliVolts()). Without a documented
// transmitter ceiling to anchor against the way pH's 0-3.0V spec allows,
// this instead uses the ESP32 ADC_11db input's own practical ceiling as the
// conservative bound - not an assumption about what the module itself should
// output. EC's own calibration anchor (Calibration.h: 1.799V for a 12.88
// mS/cm reference solution, far above the real 1.2-2.0 mS/cm cultivation
// range) sits well clear of this margin on the low side already.
constexpr int EC_FAULT_RAIL_LOW_MV = 50;
constexpr int EC_FAULT_RAIL_HIGH_MV = 3200;

// Raw 12-bit ADC count corroboration (AnalogSampler::rawMedian(), 0-4095) -
// the CHIP-INDEPENDENT rail signal, preferred over the millivolt thresholds
// above where the two disagree. analogReadMilliVolts()'s calibrated ceiling
// at raw=4095 depends on this specific chip's own eFuse calibration data
// (Two Point / Vref / Default), which is not read out or logged anywhere in
// this firmware and is known to vary unit-to-unit - commonly landing
// somewhere in the ~2900-3300mV band, but not a guaranteed fixed number. A
// millivolt-only high-rail check therefore risks a false NEGATIVE on a chip
// whose calibration curve maps raw=4095 to a value below PH_FAULT_RAIL_HIGH_MV/
// EC_FAULT_RAIL_HIGH_MV - a genuinely saturated input that never gets
// flagged. Raw count has no such risk: 4095 at 12-bit resolution is the
// ADC's own physical ceiling on every ESP32 unit, true by construction, not
// by calibration. Shared between pH and EC (both use the same 12-bit
// resolution/ADC_11db attenuation, set once in SensorManager::begin() -
// this is an ADC/attenuation-level property, not a per-sensor one, unlike
// the millivolt margins above which are legitimately sensor-specific).
//
// Margins: 5 counts (~0.12% of full scale) off each hard rail, wide enough
// to absorb residual ADC dither at true saturation even after the existing
// 51/61-sample median, narrow enough that no plausible mid-range signal
// (pH or EC alike) can ever wander into it. Combined with the millivolt
// checks via OR - either signal alone is sufficient evidence of a fault,
// see SensorManager::readPH()/readEC().
constexpr int ADC_FAULT_RAW_LOW = 5;
constexpr int ADC_FAULT_RAW_HIGH = 4090;

// How often the fault detector re-evaluates the rolling median, deliberately
// independent of the 20ms raw sample rate feeding phSampler/ecSampler and of
// STABILITY_SAMPLE_INTERVAL_MS/PH_STEP_SAMPLE_INTERVAL_MS above (different
// purposes, different cadences). Evaluating any faster would let "N
// consecutive evaluations" be satisfied by re-checking one still-settling
// median within milliseconds - the same class of bug already fixed
// elsewhere for the pH step filter and the HC-SR04 step filter (both
// require genuinely distinct, time-separated observations).
constexpr unsigned long PH_FAULT_CHECK_INTERVAL_MS = 1000UL;
constexpr unsigned long EC_FAULT_CHECK_INTERVAL_MS = 1000UL;

// Consecutive rail-condition evaluations (at the interval above) required
// before a probable hardware fault is CONFIRMED. Deliberately conservative:
// actuator switching (peristaltic pumps, solenoid, Peltier) can transiently
// disturb the analog front end, and a fault call blocks dosing/fogging until
// manually reset - a false positive here is costly, so this trades speed for
// certainty. 8 x 1000ms = 8 seconds minimum of sustained rail-pinned signal
// before a fault is ever raised.
constexpr uint8_t PH_FAULT_CONFIRM_COUNT = 8;
constexpr uint8_t EC_FAULT_CONFIRM_COUNT = 8;

// Consecutive PLAUSIBLE (non-rail) evaluations required before a CONFIRMED
// fault clears. Symmetric with the confirmation count above, same reasoning
// SensorManager::readDHT() already established for its own dhtRecoveryStreak:
// a marginal/flickering fault recovering on one isolated good sample and
// immediately failing again must not thrash phFault/ecFault (and the alert/
// notification it feeds). Clearing this flag only stops forcing sensors.ph/
// ec to NaN - it does NOT itself restore automation trust; the existing
// step filter/stability window (reset at fault onset) must still
// independently re-earn a confirmed, stable reading before
// canDosePH()/canDoseEC()/canFog() report SAFE again. See this task's
// required recovery sequence: electrical signal plausible -> fault recovery
// confirmed -> normal stability criteria satisfied -> automation eligible.
constexpr uint8_t PH_FAULT_RECOVERY_COUNT = 8;
constexpr uint8_t EC_FAULT_RECOVERY_COUNT = 8;

// Throttle for updateStabilityWindow()'s periodic diagnostic dump (real-
// hardware pre-integration follow-up, Part A) - independent of
// STABILITY_SAMPLE_INTERVAL_MS (how often the window itself re-evaluates).
// The existing "unstable; keeping last=" log only fires once, on the
// stable->unstable transition edge, so a window that then never re-agrees
// produces no further evidence on its own; this makes the current
// candidate/min/max/range visible every few seconds regardless of outcome.
constexpr unsigned long STABILITY_DIAGNOSTIC_INTERVAL_MS = 5000UL;

// Throttle for AutomationManager::logAutomationTestBlockReason() (real-
// hardware pre-integration follow-up, Part E) - the isolated
// Automation-Test-Mode "why isn't this controller acting" diagnostic.
constexpr unsigned long AUTO_TEST_BLOCK_LOG_INTERVAL_MS = 5000UL;

// Throttle for readPH()'s [PH-ADC] diagnostic - compares the raw
// analogReadMilliVolts() distribution (rawMin/rawMax/rawRange) against the
// median-filtered value (rawMedian) and the resulting pH candidate
// (candidatePH). Kept permanently (not a removable bench-only diagnostic) -
// see the finalized (EMA-free) pH acquisition architecture in readPH()'s
// own comment.
constexpr unsigned long PH_ADC_DIAGNOSTIC_INTERVAL_MS = 5000UL;

// EC calibration redesign task: same reasoning and cadence as
// PH_ADC_DIAGNOSTIC_INTERVAL_MS above, for readEC()'s new "[EC-CAL]" line
// (calibrated mV, uncompensated/compensated EC, and which calibration model
// - one-point or two-point - is currently active).
constexpr unsigned long EC_ADC_DIAGNOSTIC_INTERVAL_MS = 5000UL;

// If no NEW stable window is accepted within this long of the last one,
// the held value is too old to keep trusting and sensors.ph/ec fall back to
// NaN (SENSOR_FAULT via the existing validPH()/validEC() path - see
// SafetyManager.cpp) rather than silently acting on a stale reading forever.
// A separate concern from PH_EC_CORRECTION_STALL_TIMEOUT_MS below (that one
// bounds an active correction's dosing/lock decision; this one is purely
// "has the probe stopped reporting anything trustworthy at all") - 3 minutes
// gives headroom above legitimate churn during active dosing while still
// catching a genuinely dead/disconnected probe well before "stale" would
// otherwise mean "silently wrong for a very long time."
constexpr unsigned long PH_EC_STABLE_TIMEOUT_MS = 180000UL;

constexpr float LOW_WATER_LEVEL = 20.0f;

// Compiled defaults for systemState.highWaterTemp/coolerOffTemp - the NVS-
// restore default and what a never-before-configured device boots with.
// Cooling/fogging architecture update: these are INDEPENDENTLY authoritative
// again (AutomationManager::updateCooling() no longer overwrites either from
// systemState.maxWaterTemp every tick - that overwrite was silently
// defeating their existing Firebase/NVS read-write wiring, making them look
// configurable while never actually taking effect; removed as part of this
// change). HIGH_WATER_TEMP is the PREVENTIVE automatic-cooling trigger
// (despite its name, kept unchanged deliberately - see updateCooling()'s own
// comment on why a full field/key rename was judged too risky for an
// un-compiled change), not the maximum - that role now belongs solely to
// systemState.maxWaterTemp/TARGET_MAX_WATER_TEMP (28.0C), which is also the
// separate upper safety ceiling SafetyManager::canFog() suspends fogging
// above. COOLER_OFF_TEMP is the cooling release threshold, independently
// set rather than derived - the resulting gap (26.5 - 25.5 = 1.0C) is
// smaller than the previous derived 2.5C, an intentional consequence of
// moving the trigger down while preserving the already-confirmed 25.5C
// release point; see WATER_COOLING_HYSTERESIS's own comment below for why
// that constant is no longer used to compute it.
constexpr float HIGH_WATER_TEMP = 26.5f;
constexpr float COOLER_OFF_TEMP = 25.5f;

// RETIRED - superseded by architecture update (cooling trigger/release are
// now independently-set HIGH_WATER_TEMP/COOLER_OFF_TEMP above, not one
// derived from systemState.maxWaterTemp minus this gap). Left in place,
// unmodified, only so it remains available as a documented historical
// reference for the resulting 1.0C gap's own prior value (2.5C) - not read
// by any live code path any more.
constexpr float WATER_COOLING_HYSTERESIS = 2.5f;

// ======================================================
// Pulse cooling (FILL / COOL_SOAK / FLUSH) - TEMPORARY, UNCALIBRATED
// ======================================================
// !!! NOT VALIDATED ON HARDWARE YET !!!
// These four values are conservative bench-test placeholders only, picked
// to be safe (short soak, generous confirm timeout) rather than efficient.
// None of them come from a bench measurement of this specific cooling loop's
// fill time, trapped-water cooling rate, or flush/mixing time - see the
// pulse-cooling task's own calibration procedure (log DS18B20 through a
// manual FILL, a manual Peltier-only soak, and a manual FLUSH via
// DevOptionsFragment's existing manual actuator controls) before treating
// any of these as final. Kept as their own named constants specifically so
// they are easy to find and change once that calibration is done.
//
// DEFERRED - ineffective-cooling detection: deliberately NOT implemented.
// The pulse mechanism can currently repeat FILL->COOL_SOAK->FLUSH
// indefinitely for as long as waterTemp stays above coolerOffTemp, with no
// concept of "cooling is running but not actually working" - only a genuine
// hardware confirm-timeout (COOLING_PULSE_CONFIRM_TIMEOUT_MS below) or an
// invalid DS18B20 reading currently locks the subsystem. Adding a required
// per-cycle degrees-C drop or a maximum-cycles ceiling would be arbitrary
// without first physically characterizing this specific reservoir/Peltier
// pair - do not add one without first measuring, on real hardware:
// temperature immediately before a pulse; temperature after FILL/SOAK/FLUSH
// completes; the resulting cooling rate; how many cycles this reservoir
// typically needs to recover from a real excursion; behavior under hotter
// ambient conditions than bench-tested; and whether one Peltier module can
// hold this ~10.6L working volume below 28C at all under worst-case ambient.
// Flagged here, not solved, until that data exists.
constexpr unsigned long COOLING_PULSE_FILL_DURATION_MS_TEMP = 60UL * 1000UL;
constexpr unsigned long COOLING_PULSE_SOAK_DURATION_MS_TEMP = 120UL * 1000UL;
constexpr unsigned long COOLING_PULSE_FLUSH_DURATION_MS_TEMP = 60UL * 1000UL;
// Independent hardware-timer deadline for automatic Peltier during
// COOL_SOAK = intended soak duration + this margin, mirroring
// AUTOMATIC_DOSE_DEADLINE_MARGIN_MS's exact same "backstop only fires if
// loop() couldn't apply the on-time software stop" reasoning for pumps.
constexpr unsigned long COOLING_PULSE_SOAK_DEADLINE_MARGIN_MS = 30UL * 1000UL;
// Safety bound (not a tuning value): how long the pulse state machine waits
// for a commanded circulation/Peltier transition to physically confirm
// before treating it as a stall and locking the cooling subsystem via the
// existing coolingSubsystemLocked/Reset-Safety mechanism.
constexpr unsigned long COOLING_PULSE_CONFIRM_TIMEOUT_MS = 30UL * 1000UL;

// ======================================================
// Target (acceptable) ranges
// ======================================================
// These answer "is the reading inside the range the crop should be kept in?"
// and are what Monitoring, alerts and Reports classify against.
//
// They are deliberately SEPARATE from the actuator control thresholds below
// (HIGH_AIR_TEMP/AIR_TEMP_RELEASE, HIGH_WATER_TEMP/COOLER_OFF_TEMP,
// REFILL_START_LEVEL/REFILL_STOP_LEVEL), which answer a different question:
// "when should a fan/cooler/valve switch state?" A release/off threshold is
// hysteresis, never a target minimum.
// CONFIRMED BUG FIX (air-temperature range correction): TARGET_MAX_AIR_TEMP
// was 28.0C, stale - that number belongs to nutrient-solution temperature
// (TARGET_MAX_WATER_TEMP below, a completely separate constant/concept) and
// had been mistakenly carried into air temperature's own target range. The
// confirmed official Basilience monitored air-temperature range is 20-32C,
// matching the existing fan-control ceiling (HIGH_AIR_TEMP) and fog-strategy
// ceiling (HOT_FOG_TEMPERATURE) that were already 32.0C - those two were
// correct all along; only this target/alert range was out of step with them.
constexpr float TARGET_MIN_AIR_TEMP = 20.0f;
constexpr float TARGET_MAX_AIR_TEMP = 32.0f;

constexpr float TARGET_MIN_HUMIDITY = 60.0f;
constexpr float TARGET_MAX_HUMIDITY = 75.0f;

constexpr float TARGET_MIN_WATER_TEMP = 18.0f;
constexpr float TARGET_MAX_WATER_TEMP = 28.0f;

// Derived from the band the system already maintains the reservoir between
// (refill starts at 20%, stops at 75%). Kept as its own setting so the refill
// control thresholds stay independently tunable.
constexpr float TARGET_MIN_WATER_LEVEL = 20.0f;
constexpr float TARGET_MAX_WATER_LEVEL = 75.0f;

constexpr float HIGH_AIR_TEMP = 32.0f;
constexpr float AIR_TEMP_RELEASE = 26.0f;
constexpr float HIGH_HUMIDITY = 75.0f;
constexpr float HUMIDITY_RELEASE = 70.0f;

// Canopy Fan's own cold-side control pair, symmetric with
// HIGH_AIR_TEMP/AIR_TEMP_RELEASE above: demand engages below LOW_AIR_TEMP
// and releases only once temperature has recovered to COLD_AIR_RELEASE (2C
// above the trigger, matching HIGH_AIR_TEMP/AIR_TEMP_RELEASE's own 2C gap),
// not merely back at LOW_AIR_TEMP - the same latch-with-hysteresis shape,
// to avoid rapid 50%/70% toggling right at the boundary. Deliberately its
// own control pair rather than the app-editable minAirTemp/maxAirTemp
// target-range fields, for the same reason HOT uses HIGH_AIR_TEMP/
// AIR_TEMP_RELEASE instead of maxAirTemp: control thresholds ("when does
// equipment switch") and target ranges ("what counts as an acceptable
// reading") are deliberately separate concepts throughout this codebase.
constexpr float LOW_AIR_TEMP = 20.0f;
constexpr float COLD_AIR_RELEASE = 22.0f;

// Fog-strategy cadence selector thresholds (DHT22 air temperature only -
// never nutrient-solution temperature, never humidity). Sole consumer is
// AutomationManager::processFogCycle(), via systemState.hotFogTemperature/
// coldFogTemperature in Types.h, which default to these constants. Unlike
// highAirTemp/highHumidity above, this pair has no Firebase sync, no NVS
// persistence, and no app UI yet - changing the compiled default here is
// currently the only way to change it. Boundary is inclusive on both ends:
// temp >= HOT_FOG_TEMPERATURE -> hot cadence, temp <= COLD_FOG_TEMPERATURE
// -> cold cadence, otherwise normal.
constexpr float HOT_FOG_TEMPERATURE = 32.0f;
constexpr float COLD_FOG_TEMPERATURE = 20.0f;

// ======================================================
// EC Sampling
// ======================================================

constexpr uint8_t EC_SAMPLE_COUNT = 61;
constexpr unsigned long EC_SAMPLE_INTERVAL = 20;

// ======================================================
// pH Sampling
// ======================================================

constexpr uint8_t PH_SAMPLE_COUNT = 51;
constexpr unsigned long PH_SAMPLE_INTERVAL = 20;

// ======================================================
// Timing
// ======================================================

constexpr unsigned long MIXING_DURATION =
    60UL * 1000UL;
constexpr unsigned long PH_DOSE_COOLDOWN =
    60000UL;
constexpr unsigned long EC_DOSE_COOLDOWN =
    60000UL;

// ======================================================

// Hard cap on the SENSOR_STABILIZATION boot state (see
// AutomationManager::handleSensorStabilization()). That state now exits as
// soon as pH/EC actually confirm stable (isPhCurrentlyStable()/
// isEcCurrentlyStable()), not on a blind timer - this is only the fallback
// so a genuinely stuck/disconnected probe still reaches STARTUP eventually
// instead of hanging forever. 1 minute still comfortably covers
// PH_EC_ANALOG_SETTLE_TIME (20s) plus the pH step filter's confirmation
// window with margin for a slow-to-settle probe.
constexpr unsigned long SENSOR_STABILIZATION_TIME = 60000UL; // 1 minute

// Coherent-snapshot readiness (quick-response refinement task) - deliberately
// NOT SENSOR_STABILIZATION_TIME above, which is AutomationManager's own
// boot-wait state duration for a different purpose (holding automatic
// refill/pH/EC/fog regulation off) and is far longer than Monitoring UI
// readiness should ever need to wait. See FirebaseManager::writeSensors()'s
// own comment for the exact readiness rule: the "fast" sensors (water level,
// pH telemetry) reaching their own first determination before
// SENSOR_READY_MIN_MS is not trusted as coincidence (an artifact of
// evaluating before any real read cycle has run), and SENSOR_READY_MAX_MS is
// the hard fallback so a genuinely stuck/failed fast sensor still bounds
// readiness rather than blocking the dashboard indefinitely.
constexpr unsigned long SENSOR_READY_MIN_MS = 500UL;
constexpr unsigned long SENSOR_READY_MAX_MS = 3000UL;

// How long after physical pH/EC probes become the active source their
// readings are held invalid (NaN) rather than published/acted on. The
// analog front end (glass-electrode buffer, coupling caps) needs time to
// charge after power/reconnection; until then the ADC returns a real,
// smoothly-drifting-but-wrong value that would otherwise trip alerts and
// trigger dosing mid-ramp. 20s covers the settling ramp observed on field
// serial logs (~17s from cold power-on to a stable reading) with margin -
// re-tune against your own probe/hardware if it settles slower/faster.
constexpr unsigned long PH_EC_ANALOG_SETTLE_TIME = 20000UL;

constexpr unsigned long STARTUP_ON_TIME =
    90UL * 1000UL; // 1 minute 30 seconds

constexpr unsigned long STARTUP_OFF_TIME =
    60UL * 1000UL; // 1 minute

constexpr unsigned long NORMAL_FOG_ON_TIME =
    5UL * 60UL * 1000UL; // 5 minutes

constexpr unsigned long NORMAL_FOG_OFF_TIME =
    5UL * 60UL * 1000UL; // 5 minutes

constexpr unsigned long HOT_FOG_ON_TIME =
    8UL * 60UL * 1000UL; // 8 minutes

constexpr unsigned long HOT_FOG_OFF_TIME =
    4UL * 60UL * 1000UL; // 4 minutes

constexpr unsigned long COLD_FOG_ON_TIME =
    3UL * 60UL * 1000UL; // 3 minutes

constexpr unsigned long COLD_FOG_OFF_TIME =
    5UL * 60UL * 1000UL; // 5 minutes

// Short Blower overrun after the Fogger turns off (automatic fogging and
// startup fogging alike) to clear fog concentrated near the reservoir toward
// the root chamber. Consumes the front of the existing OFF/rest window -
// never extends NORMAL/HOT/COLD/STARTUP's total cycle length.
constexpr unsigned long BLOWER_PURGE_MS =
    30UL * 1000UL; // 30 seconds

// Configurable automatic root-zone Blower speed while the Fogger/Blower
// pair is actively ON (NORMAL/COLD/HOT fogging cadence - see
// AutomationManager::processFogCycle()). CONFIRMED BUG FIX (root-blower/
// canopy-fan speed separation): this constant and systemState.
// blowerSpeedPercent already existed, fully wired to Firebase/NVS, but were
// never actually read by processFogCycle() - it used lastAutomaticCanopySpeed
// (the CANOPY_FAN's own temp/humidity-derived speed) instead, despite this
// comment already (incorrectly) claiming the replacement had happened. Now
// genuinely consumed, and the root-zone blower's automatic fogging speed is
// independent of canopy temperature/humidity demand and of CANOPY_FAN's PWM,
// exactly as intended - confirmed default is 65%, not the previous 30%
// (the comment previously and inconsistently also said "50%" here - neither
// matched the actual 30 value; this is now internally consistent). Used as
// the DEFAULT AND the boot/never-configured fallback - it is never written
// back to Firebase on its own; see FirebaseManager::readSettings()'s own
// comment for the accept/reject rule. The purge phase (BLOWER_PURGE_MS)
// remains a separate, deliberately fixed 100% - never this value - see
// processFogCycle()'s own comment. Range mirrors validPercentage()-style
// bounds but narrower, since a fogging airflow test below 30% is not a
// realistic operating point.
constexpr uint8_t BLOWER_SPEED_DEFAULT_PERCENT = 65;
constexpr uint8_t BLOWER_SPEED_MIN_PERCENT = 30;
constexpr uint8_t BLOWER_SPEED_MAX_PERCENT = 100;

// Canopy/Blower PWM hardware parameters - see ActuatorManager::begin()'s
// ledcAttach() call, the single place these are applied, and
// ActuatorManager::percentToDuty(), the single place a 0-100% command is
// converted to a duty value. Named here (rather than the previous inline
// 5000/8 literals) so the max-duty calculation shared by percentToDuty()
// and its own diagnostic logging has one source of truth.
//
// Frequency lowered from 5000 Hz to 200 Hz after real-hardware bench testing
// (FanPwmSpeedTest.ino) on the actual driver board, an opto-isolated 4-channel
// MOSFET module. At 5000 Hz (200us/cycle) the optocoupler's turn-on/turn-off
// delay ate a large share of every pulse: low percentages barely switched on
// and anything a bit higher never fully switched off, so nearly the whole
// 0-100% range collapsed to full speed. At 200 Hz (5ms/cycle) the fans
// respond proportionally across the range - confirmed usable from 15%
// (Blower) / 25% (Canopy Fan) up to 100%, with 65-75% the cleanest-running
// band for both.
constexpr uint32_t CANOPY_BLOWER_PWM_FREQUENCY_HZ = 200;
constexpr uint8_t CANOPY_BLOWER_PWM_RESOLUTION_BITS = 8;

// Initial circulation-only period after a dose, before the first checkpoint
// is even considered - lets the newly-dosed chemical actually mix through
// the reservoir before a reading means anything. Retimed from 60000UL: 30s
// post-dose circulation + a further 60s silent read, per the quiet-
// monitoring/4-minute-budget redesign. phStabilizationCirculationConfirmedAt/
// ecStabilizationCirculationConfirmedAt (AutomationManager) still measure
// from confirmed circulation, not state entry - see their own comments.
constexpr unsigned long PH_STABILIZATION_TIME = 90000UL;
constexpr unsigned long EC_STABILIZATION_TIME = 90000UL;

// Past the initial PH_STABILIZATION_TIME/EC_STABILIZATION_TIME window, this
// is the trend re-sample interval AutomationManager::handleStabilizingPH()/
// handleStabilizingEC() use to tell "still improving on its own," "stalled,"
// and "reversing" apart, rather than blindly redosing on a timer. See the
// quiet-monitoring/4-minute-budget redesign.
constexpr unsigned long PH_EC_RECHECK_INTERVAL_MS = 30000UL;

constexpr unsigned long PH_DOSING_TIME = 5000UL;
constexpr unsigned long EC_DOSING_TIME = 5000UL;
constexpr unsigned long EC_DILUTION_TIME = 5000UL;

// Whole-correction time budget: replaces the old MAX_PH_ATTEMPTS/
// MAX_EC_ATTEMPTS=3 retry-count limit as the trigger for locking a stalled
// subsystem. Anchored to systemState.correctionCycleStartAt, set once when
// a correction first begins and not reset by an internal redose - see
// AutomationManager::handleStabilizingPH()/handleStabilizingEC(). A HARD
// ceiling on the whole episode: once expired, the correction locks
// (phSubsystemLocked/ecSubsystemLocked, requiring manual Reset Safety)
// unless the target was already reached first - see the target-reached
// check earlier in the same function, which returns via
// completeCurrentOperation() independent of this budget. CONFIRMED BUG FIX
// (correction-budget limbo): this previously locked ONLY when the reading
// was also classified as not improving (phLastTrendImproving/
// ecLastTrendImproving == false), which let a reading still classified as
// improving - even glacially, via passive drift with no redose ever
// actually landing it in range - remain parked in STABILIZING_PH/
// STABILIZING_EC indefinitely: dosing correctly stopped (both redose paths
// are separately gated on the budget too), but reservoirLocked stayed true
// and the state never returned to NORMAL, permanently blocking the OTHER
// chemical subsystem from ever being evaluated. Trend classification still
// guides which redose path fires WHILE under budget; it no longer has any
// bearing on whether the episode terminates once the budget expires. The
// existing phOutOfRange/ecLow/ecHigh alert path is unaffected either way -
// it re-evaluates off sensors.ph/ec every tick regardless of correction
// state, so monitoring and notification continue normally after the lock.
constexpr unsigned long PH_EC_CORRECTION_STALL_TIMEOUT_MS = 240000UL; // 4 min

// A reading must hold continuously stable this long, past the first
// checkpoint, before it is trusted enough to publish to Firebase and to end
// a correction on. Longer than SensorManager's own stability window
// (STABILITY_SAMPLE_WINDOW * STABILITY_SAMPLE_INTERVAL_MS = 10s) - that
// window says "not currently moving," this says "stayed that way."
constexpr unsigned long PH_EC_STABLE_HOLD_FOR_PUBLISH_MS = 25000UL;

// Minimum change between two trend samples (PH_EC_RECHECK_INTERVAL_MS apart)
// to count as real movement rather than probe/ADC noise. Starting values -
// expect to tune after watching real dosing, same as every other tolerance
// in this file.
constexpr float PH_TREND_NOISE_FLOOR = 0.03f;
constexpr float EC_TREND_NOISE_FLOOR = 0.02f;

// ======================================================
// Independent Automatic-Dose/Refill Deadline (esp_timer, ActuatorManager)
// ======================================================
// Critical verification report, Priority 3/4: automatic PH_UP_PUMP/
// PH_DOWN_PUMP/GROW_PUMP/BLOOM_PUMP dosing and automatic SOLENOID
// refill/dilution runs are now armed with the SAME independent esp_timer
// deadline mechanism that already protects manual commands (see
// ActuatorManager::isAutomaticDeadlineProtected()/automaticDeadlineMs()),
// so a stalled loop() (e.g. a blocking Firebase reconnect) can never leave
// a dosing pump or the refill solenoid physically energized past this
// margin beyond its own intended duration. Deliberately LONGER than every
// intended automatic duration it applies to (PH_DOSING_TIME, EC_DOSING_TIME,
// EC_DILUTION_TIME, AUTOMATIC_REFILL_RUN_TIME) so AutomationManager's own
// on-time millis() check is always given the first opportunity to stop the
// actuator normally under ordinary (non-stalled) operation - this deadline
// only ever fires as the backstop for a loop that could not return in time.
constexpr unsigned long AUTOMATIC_DOSE_DEADLINE_MARGIN_MS = 5000UL;
// ======================================================

constexpr int OUT_OF_RANGE_REQUIRED = 3;

// ======================================================
// Debug
// ======================================================

constexpr bool DEBUG_ENABLED = true;
constexpr unsigned long DEBUG_INTERVAL = 2000UL;

// ======================================================
// Water Refill (LEGACY percentage model - superseded)
// ======================================================
// REFILL_START_LEVEL/REFILL_STOP_LEVEL and systemState.refillStartLevel/
// refillStopLevel are no longer read by any control path (SafetyManager,
// ActuatorManager, AlertManager, AutomationManager all switched to the
// centimeter-based thresholds in the "Water Reservoir Geometry" section
// below - see the water-depth-model task report). Left in place,
// unmodified, only so existing NVS/RTDB data and any external reader of
// /settings/refillStartLevel|refillStopLevel are not silently broken.
constexpr float REFILL_START_LEVEL = 20.0f;

constexpr float REFILL_STOP_LEVEL = 75.0f;

// Temporary bounded automatic-refill test policy. Each automatic attempt may
// run the solenoid for at most 30 seconds, then waits five seconds for the
// ultrasonic reading to settle before evaluating the runtime refillStopLevelCm.
// Manual refill commands retain their existing OperationRequest timeout.
constexpr uint8_t MAX_REFILL_ATTEMPTS = 3;
constexpr unsigned long AUTOMATIC_REFILL_RUN_TIME = 30UL * 1000UL;
constexpr unsigned long AUTOMATIC_REFILL_SETTLE_TIME = 10UL * 1000UL;

constexpr unsigned long MANUAL_PUMP_RUNTIME = 5000UL;

// Manual Mode automatically expires after this long with no legitimate
// manual interaction (enabling Manual Mode, a fresh actuator command, or a
// fresh REFILL/RESET_SAFETY operation request - see
// FirebaseManager::lastManualCommandActivityAt and
// ActuatorManager::update()'s expiry check). Enforced locally via millis(),
// independent of Firebase/Wi-Fi connectivity, so it still fires if the app
// closes, the Admin logs out, or the device goes offline. Does not extend
// or shorten any actuator's own independent deadline above
// (MANUAL_PUMP_RUNTIME/OPERATION_TIMEOUT_MS) - those remain authoritative.
constexpr unsigned long MANUAL_MODE_INACTIVITY_TIMEOUT_MS = 15UL * 60UL * 1000UL;

// ======================================================
// Water Level Sensor Calibration
// ======================================================
// Ultrasonic (HC-SR04) distance, in cm, from the sensor to the reservoir
// BOTTOM - not a universal constant, it depends on where the sensor is
// physically mounted, so this is only the firmware default;
// systemState.waterLevelEmptyDistanceCm (settable via /settings, see
// FirebaseManager) is what SensorManager::readWaterLevel() actually uses,
// so a mismatched installation can be corrected without a reflash.
// Measured value for the current reservoir (see the water-depth-model task
// report): 28.67cm.
constexpr float WATER_LEVEL_EMPTY_DISTANCE_CM = 28.67f;

// LEGACY - no longer consumed by readWaterLevel()'s depth/percent/liters
// formula (see "Water Reservoir Geometry" below, which uses the fixed
// MAX_WORKING_WATER_CM instead of a second configurable "full" distance).
// Left in place, unmodified, for the same non-destructive reason as
// REFILL_START_LEVEL/REFILL_STOP_LEVEL above.
constexpr float WATER_LEVEL_FULL_DISTANCE_CM = 5.0f;

// ======================================================
// Water Reservoir Geometry (authoritative water-depth model)
// ======================================================
// Replaces the old "distance mapped linearly between an empty-distance and
// a full-distance" percentage model - see the water-depth-model task
// report. These are the reservoir's actual measured physical dimensions,
// fixed for this reservoir design - unlike WATER_LEVEL_EMPTY_DISTANCE_CM
// (sensor mounting height, which does vary per installation), they are not
// exposed as a /settings field. The physical container height (~29cm) must
// NOT be treated as 100% - MAX_WORKING_WATER_CM (6.0cm) is the working
// capacity, matching the intended operating band, not the tank's full
// physical depth.
//
// Reference: depth 0/1/2/3/4/5/6 cm -> 0/16.7/33.3/50.0/66.7/83.3/100 % ->
// 0.00/1.77/3.54/5.30/7.07/8.84/10.61 L.
constexpr float RESERVOIR_LENGTH_CM = 52.0f;
constexpr float RESERVOIR_WIDTH_CM = 34.0f;
constexpr float MAX_WORKING_WATER_CM = 6.0f;

// Control thresholds (centimeters of water DEPTH) - authoritative for
// automation. Do not derive refill decisions by converting the working
// percentage back into a level; compare sensors.waterLevelCm directly.
//
// Hierarchy (see the static automation integration audit - CONFIRMED
// current design, corrected there from an earlier revision that had this
// backwards):
//   > REFILL_START_CM         : normal water-dependent operation allowed.
//   <= REFILL_START_CM (2.0)  : OPERATIONAL low water - refill becomes
//                                eligible to start AND pH/EC dosing,
//                                fogging, and cooling are blocked (the same
//                                role systemState.refillStartLevel/the old
//                                percentage model used to play - see
//                                SafetyManager::canDosePH()/canDoseEC()/
//                                canFog()/canCool() and
//                                ActuatorManager::lowWaterBlocks()).
//   <= CRITICAL_LOW_WATER_CM (1.0): a stricter, SEPARATE escalation on top
//                                of the above - not a replacement for it.
//                                Not yet wired to a distinct behavior beyond
//                                being its own configurable threshold; add
//                                one (e.g. a dedicated alert) if a harder
//                                response than the operational block above
//                                is ever required.
// REFILL_STOP_CM is deliberate hysteresis above REFILL_START_CM so a
// completed refill is not immediately re-triggered by the same low reading;
// 6cm (MAX_WORKING_WATER_CM) is never the refill target, only the
// working-capacity ceiling for monitoring/reporting - and is never a cap on
// the actual measured sensors.waterLevelCm itself (see
// SensorManager::readWaterLevel()): only the derived percentage clamps at
// 100%, an overfilled reservoir still reports its true depth above 6cm.
constexpr float CRITICAL_LOW_WATER_CM = 1.0f;
constexpr float REFILL_START_CM = 2.0f;
constexpr float REFILL_STOP_CM = 3.0f;

// ======================================================
// HC-SR04 Accepted-Value Temporal Plausibility Filter
// ======================================================
// Second stage, applied AFTER the existing median-of-5 (SensorManager::
// readWaterLevel()) - see the automation resilience pass report. The median
// alone still lets a run of consecutive false echoes (not just one outlier)
// shift the candidate depth by an implausible amount in a single tick, e.g.
// the observed 4.03cm -> 1.70cm -> 4.03cm with no real water movement. This
// stage holds the previous ACCEPTED depth steady against any single-tick
// jump larger than WATER_LEVEL_STEP_ACCEPT_CM until WATER_LEVEL_STEP_CONFIRM_
// COUNT consecutive post-median candidates agree with each other within
// WATER_LEVEL_STEP_CONFIRM_TOLERANCE_CM - only then is the new level trusted
// enough to become the accepted control value. A single bad echo surrounded
// by consistent real readings never accumulates 3 agreeing candidates and is
// permanently rejected; a genuine drain/fill/overfill still confirms within
// a few read cycles (each WATER_LEVEL_READ_INTERVAL_MS = 5s apart, so up to
// ~10s worst case for 3 agreeing candidates - corrected from an earlier,
// incorrect "~300ms apart" figure that belonged to pH's own, separate
// PH_STEP_SAMPLE_INTERVAL_MS and did not describe this filter).
constexpr float WATER_LEVEL_STEP_ACCEPT_CM = 0.40f;
constexpr float WATER_LEVEL_STEP_CONFIRM_TOLERANCE_CM = 0.15f;
constexpr uint8_t WATER_LEVEL_STEP_CONFIRM_COUNT = 3;
#endif
