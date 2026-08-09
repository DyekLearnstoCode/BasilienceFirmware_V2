#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "Version.h"

#define OPERATION_TIMEOUT_MS 300000UL

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
// Sensor Thresholds
// ======================================================

constexpr float MIN_HUMIDITY = 60.0f;
constexpr float MAX_HUMIDITY = 80.0f;

constexpr float MIN_PH = 5.5f;
constexpr float MAX_PH = 6.5f;

constexpr float MIN_EC = 1.2f;

constexpr float LOW_WATER_LEVEL = 20.0f;

constexpr float HIGH_WATER_TEMP = 25.0f;
constexpr float COOLER_OFF_TEMP = 22.5f;

constexpr float HIGH_AIR_TEMP = 35.0f;
constexpr float LOW_AIR_TEMP = 28.0f;

constexpr float HOT_FOG_TEMPERATURE = 30.0f;
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
    30UL * 1000UL; // 30 seconds

constexpr unsigned long NORMAL_FOG_OFF_TIME =
    2UL * 60UL * 1000UL; // 2 minutes

constexpr unsigned long HOT_FOG_ON_TIME =
    15UL * 1000UL; // 15 seconds

constexpr unsigned long HOT_FOG_OFF_TIME =
    60UL * 1000UL; // 1 minute

constexpr unsigned long COLD_FOG_ON_TIME =
    60UL * 1000UL; // 1 minute

constexpr unsigned long COLD_FOG_OFF_TIME =
    4UL * 60UL * 1000UL; // 4 minutes

constexpr unsigned long PH_STABILIZATION_TIME = 10000UL;
constexpr unsigned long EC_STABILIZATION_TIME = 10000UL;

constexpr unsigned long PH_DOSING_TIME = 5000UL;
constexpr unsigned long EC_DOSING_TIME = 5000UL;

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
