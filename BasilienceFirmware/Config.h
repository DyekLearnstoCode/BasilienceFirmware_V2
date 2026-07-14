#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======================================================
// WiFi
// ======================================================

#define WIFI_SSID "EVITH WIFI"
#define WIFI_PASSWORD "Ronald123"

// ======================================================
// Firebase
// ======================================================

#define API_KEY "AIzaSyDaJ7F8tAREnCo7zrrY_sJ6SgfNuYQtra0"

#define DATABASE_URL \
    "https://basilience-database-default-rtdb.asia-southeast1.firebasedatabase.app"

// ======================================================
// GPIO PINS
// ======================================================

#define FOGGER_PIN 26
#define BLOWER_PIN 27
#define GROW_LIGHT_PIN 25
#define NUTRIENT_PUMP_PIN 33
#define PH_UP_PIN 14
#define PH_DOWN_PIN 12
#define CANOPY_FAN_PIN 13

#define DHT_PIN 4
#define DHTTYPE DHT22
#define WATER_TEMP_PIN 5
#define EC_PIN 34
#define PH_SENSOR_PIN 35

#define TRIG_PIN 18
#define ECHO_PIN 19

#define SOLENOID_PIN 15
#define PELTIER_PIN 16
#define WATER_HEATER_PIN 17

// ======================================================
// EC Sensor Calibration
// ======================================================

constexpr float ADC_REFERENCE = 3.3f;
constexpr int ADC_RESOLUTION = 4095;
constexpr float EC_FACTOR = 1.106f;

// ======================================================
// Sensor Thresholds
// ======================================================

constexpr float MIN_HUMIDITY = 60.0f;
constexpr float MAX_HUMIDITY = 80.0f;

constexpr float MIN_PH = 5.5f;
constexpr float MAX_PH = 6.5f;

constexpr float MIN_EC = 1.2f;

constexpr float LOW_WATER_LEVEL = 20.0f;

constexpr float HIGH_WATER_TEMP = 24.0f;
constexpr float LOW_WATER_TEMP = 22.0f;

constexpr float HIGH_AIR_TEMP = 35.0f;
constexpr float LOW_AIR_TEMP = 28.0f;

// ======================================================
// EC Sampling
// ======================================================

constexpr uint8_t EC_SAMPLE_COUNT = 60;

constexpr unsigned long EC_SAMPLE_INTERVAL = 20;

// ======================================================
// pH Sampling
// ======================================================

constexpr uint8_t PH_SAMPLE_COUNT = 50;

constexpr unsigned long PH_SAMPLE_INTERVAL = 20;

// ======================================================
// pH Calibration
// ======================================================

constexpr float PH_SLOPE = -0.00571715f;
constexpr float PH_OFFSET = 21.535f;

// ======================================================
// Timing
// ======================================================

constexpr unsigned long STARTUP_FOG_TIME =
    20UL * 60UL * 1000UL;

constexpr unsigned long STARTUP_REST_TIME =
    10UL * 60UL * 1000UL;

constexpr unsigned long MIXING_DURATION =
    60UL * 1000UL;

constexpr unsigned long PH_DOSE_COOLDOWN =
    60000UL;

constexpr unsigned long EC_DOSE_COOLDOWN =
    60000UL;

// ======================================================

constexpr int OUT_OF_RANGE_REQUIRED = 3;

// ======================================================
// Debug
// ======================================================

constexpr bool DEBUG_ENABLED = true;

constexpr unsigned long DEBUG_INTERVAL = 1000UL;

#endif