#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "Version.h"

#define OPERATION_TIMEOUT_MS 300000UL

// Bounded local fallback: how long Fogger/Blower resume waits after a local
// pH/EC correction completes for RTDB COMPLETED publication before releasing
// from local safe state anyway, so plant control never depends on cloud
// availability.
constexpr unsigned long CHEMISTRY_FOGGING_HOLD_TIMEOUT_MS = 30000UL;

// Minimum spacing between completed DS18B20 conversions. The sensor read
// itself is a blocking OneWire transaction, so it must not run every loop
// iteration - both to stop it from dominating loop() timing and to reduce
// how often it can collide with other blocking work (e.g. Firebase calls).
constexpr unsigned long WATER_TEMP_READ_INTERVAL_MS = 1000UL;

// Minimum spacing between HC-SR04 trigger pulses. Without this, readWaterLevel()
// re-triggers on literally every loop iteration - far faster than the sensor's
// own echo/reverberation settling time - which is a common cause of spurious
// pulseIn() timeouts unrelated to the sensor or wiring actually failing.
constexpr unsigned long WATER_LEVEL_READ_INTERVAL_MS = 300UL;

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
constexpr unsigned long DHT_READ_INTERVAL_MS = 2500UL;

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

// No single fixed baud here: unlike the previous module, SIM800L's actual
// UART rate on a given board isn't known in advance (varies by unit/firmware
// and isn't queryable without already talking to it at that rate), so
// GsmManager::WAITING_FOR_MODULE probes a short list of candidate bauds
// (GsmManager.h: BAUD_CANDIDATES) instead of assuming one. See GsmManager.h.

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
constexpr float EC_STABILITY_TOLERANCE = 0.05f;

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

// If no NEW stable window is accepted within this long of the last one,
// the held value is too old to keep trusting and sensors.ph/ec fall back to
// NaN (SENSOR_FAULT via the existing validPH()/validEC() path - see
// SafetyManager.cpp) rather than silently acting on a stale reading forever.
// 3 minutes gives generous headroom above the worst-case legitimate churn
// during active dosing (MAX_PH_ATTEMPTS/MAX_EC_ATTEMPTS=3 retries, each up to
// ~15s dose + PH_STABILIZATION_TIME/EC_STABILIZATION_TIME=10s stabilize =~75s
// total) while still catching a genuinely dead/disconnected probe well
// before "stale" would otherwise mean "silently wrong for a very long time."
constexpr unsigned long PH_EC_STABLE_TIMEOUT_MS = 180000UL;

constexpr float LOW_WATER_LEVEL = 20.0f;

// Compiled-default fallback for systemState.highWaterTemp/coolerOffTemp -
// used only before the first AutomationManager::updateCooling() tick derives
// them from systemState.maxWaterTemp, and as the NVS-restore default. Not
// the authoritative cooling threshold; see WATER_COOLING_HYSTERESIS.
constexpr float HIGH_WATER_TEMP = 25.0f;
constexpr float COOLER_OFF_TEMP = 22.5f;

// Effective cooling hysteresis: the app-configured maxWaterTemp is now the
// single authoritative cooling-ON ceiling (updateCooling() applies
// waterTemp > maxWaterTemp), and this is subtracted from it for the
// cooling-OFF release threshold (waterTemp < maxWaterTemp -
// WATER_COOLING_HYSTERESIS) - preserving the original 25.0/22.5 = 2.5C gap
// as a single named constant rather than two independently configurable
// thresholds.
constexpr float WATER_COOLING_HYSTERESIS = 2.5f;

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
constexpr float TARGET_MIN_AIR_TEMP = 20.0f;
constexpr float TARGET_MAX_AIR_TEMP = 28.0f;

constexpr float TARGET_MIN_HUMIDITY = 60.0f;
constexpr float TARGET_MAX_HUMIDITY = 75.0f;

constexpr float TARGET_MIN_WATER_TEMP = 18.0f;
constexpr float TARGET_MAX_WATER_TEMP = 25.0f;

// Derived from the band the system already maintains the reservoir between
// (refill starts at 20%, stops at 75%). Kept as its own setting so the refill
// control thresholds stay independently tunable.
constexpr float TARGET_MIN_WATER_LEVEL = 20.0f;
constexpr float TARGET_MAX_WATER_LEVEL = 75.0f;

constexpr float HIGH_AIR_TEMP = 28.0f;
constexpr float AIR_TEMP_RELEASE = 26.0f;
constexpr float HIGH_HUMIDITY = 75.0f;
constexpr float HUMIDITY_RELEASE = 70.0f;

// Canopy Fan's own cold-side control pair, symmetric with
// HIGH_AIR_TEMP/AIR_TEMP_RELEASE above: demand engages below LOW_AIR_TEMP
// and releases only once temperature has recovered to COLD_AIR_RELEASE (2C
// above the trigger, matching HIGH_AIR_TEMP/AIR_TEMP_RELEASE's own 2C gap),
// not merely back at LOW_AIR_TEMP - the same latch-with-hysteresis shape,
// to avoid rapid 30%/50% toggling right at the boundary. Deliberately its
// own control pair rather than the app-editable minAirTemp/maxAirTemp
// target-range fields, for the same reason HOT uses HIGH_AIR_TEMP/
// AIR_TEMP_RELEASE instead of maxAirTemp: control thresholds ("when does
// equipment switch") and target ranges ("what counts as an acceptable
// reading") are deliberately separate concepts throughout this codebase.
constexpr float LOW_AIR_TEMP = 20.0f;
constexpr float COLD_AIR_RELEASE = 22.0f;

constexpr float HOT_FOG_TEMPERATURE = 28.0f;
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

constexpr unsigned long SENSOR_STABILIZATION_TIME = 10000UL;

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
    180UL * 1000UL; // 3 minutes

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

// Configurable automatic Blower speed (real-hardware Canopy/Blower PWM
// follow-up). Replaces the previous hard-coded 100% used while the
// Fogger/Blower pair is automatically ON - see AutomationManager::
// processFogCycle(). 50% is a FALLBACK ONLY (used when no valid Firebase
// value has ever been accepted, or the device is offline at boot before
// settings load) - it is never written back to Firebase on its own; see
// FirebaseManager::readSettings()'s own comment for the accept/reject
// rule. Range mirrors validPercentage()-style bounds but narrower, since a
// fogging airflow test below 30% is not a realistic operating point.
constexpr uint8_t BLOWER_SPEED_DEFAULT_PERCENT = 50;
constexpr uint8_t BLOWER_SPEED_MIN_PERCENT = 30;
constexpr uint8_t BLOWER_SPEED_MAX_PERCENT = 100;

// Canopy/Blower PWM hardware parameters - see ActuatorManager::begin()'s
// ledcAttach() call, the single place these are applied, and
// ActuatorManager::percentToDuty(), the single place a 0-100% command is
// converted to a duty value. Named here (rather than the previous inline
// 5000/8 literals) so the max-duty calculation shared by percentToDuty()
// and its own diagnostic logging has one source of truth.
constexpr uint32_t CANOPY_BLOWER_PWM_FREQUENCY_HZ = 5000;
constexpr uint8_t CANOPY_BLOWER_PWM_RESOLUTION_BITS = 8;

constexpr unsigned long PH_STABILIZATION_TIME = 10000UL;
constexpr unsigned long EC_STABILIZATION_TIME = 10000UL;

constexpr unsigned long PH_DOSING_TIME = 5000UL;
constexpr unsigned long EC_DOSING_TIME = 5000UL;
constexpr unsigned long EC_DILUTION_TIME = 5000UL;

constexpr uint8_t MAX_PH_ATTEMPTS = 3;
constexpr uint8_t MAX_EC_ATTEMPTS = 3;
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
constexpr unsigned long AUTOMATIC_REFILL_SETTLE_TIME = 5UL * 1000UL;

constexpr unsigned long MANUAL_PUMP_RUNTIME = 5000UL;

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
// a few read cycles (each ~300ms apart, so ~0.6-0.9s worst case).
constexpr float WATER_LEVEL_STEP_ACCEPT_CM = 0.40f;
constexpr float WATER_LEVEL_STEP_CONFIRM_TOLERANCE_CM = 0.15f;
constexpr uint8_t WATER_LEVEL_STEP_CONFIRM_COUNT = 3;
#endif
