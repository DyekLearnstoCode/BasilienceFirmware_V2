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
// GSM / A7680C (SIMCom A76XX family)
// ======================================================
// Confirmed non-conflicting production wiring - does not overlap with any
// sensor, actuator, I2C, or UART0 (USB/debug) pin above.
constexpr uint8_t GSM_RX_PIN = 36;  // ESP32 RX <- A7680C UTX
constexpr uint8_t GSM_TX_PIN = 23;  // ESP32 TX -> A7680C URX

// A76XX default UART framing is 115200 8N1. Autobaud is supported by the
// module, but a fixed rate is used here so GsmManager's bounded timeouts
// don't also have to account for autobaud detection latency/uncertainty.
constexpr unsigned long GSM_BAUD_RATE = 115200UL;

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

constexpr float LOW_WATER_LEVEL = 20.0f;

constexpr float HIGH_WATER_TEMP = 25.0f;
constexpr float COOLER_OFF_TEMP = 22.5f;

constexpr float HIGH_AIR_TEMP = 28.0f;
constexpr float AIR_TEMP_RELEASE = 26.0f;
constexpr float HIGH_HUMIDITY = 75.0f;
constexpr float HUMIDITY_RELEASE = 70.0f;

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

constexpr unsigned long STARTUP_ON_TIME =
    60UL * 1000UL; // 1 minute

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

constexpr bool DEBUG_ENABLED = false;
constexpr unsigned long DEBUG_INTERVAL = 1000UL;

// ======================================================
// Water Refill
// ======================================================

constexpr float REFILL_START_LEVEL = 20.0f;

constexpr float REFILL_STOP_LEVEL = 75.0f;

constexpr unsigned long MANUAL_PUMP_RUNTIME = 5000UL;
#endif
