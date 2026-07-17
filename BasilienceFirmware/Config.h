#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======================================================
// WiFi
// ======================================================

//#define WIFI_SSID "EVITH WIFI"
//#define WIFI_PASSWORD "Ronald123"

#define WIFI_SSID "Deduyo's_Wifi_2.4G"
#define WIFI_PASSWORD "p@ssw0rd"

// ======================================================
// Firebase
// ======================================================

#define API_KEY "AIzaSyDaJ7F8tAREnCo7zrrY_sJ6SgfNuYQtra0"

#define DATABASE_URL \
    "https://basilience-database-default-rtdb.asia-southeast1.firebasedatabase.app"

// ======================================================
// SSR Outputs
// ======================================================

#define FOGGER_PIN 26
#define GROW_LIGHT_PIN 25

// ======================================================
// MOSFET Outputs
// ======================================================

#define BLOWER_PIN 27
#define SOLENOID_PIN 15

// ======================================================
// Peristaltic Pumps
// ======================================================

#define GROW_PUMP_PIN 32  // Temporary
#define BLOOM_PUMP_PIN 33 // Temporary

#define PH_UP_PUMP_PIN 14
#define PH_DOWN_PUMP_PIN 12

// ======================================================
// Temperature
// ======================================================

#define WATER_HEATER_PIN 17
#define PELTIER_PIN 16

// ======================================================
// Sensor Inputs
// ======================================================

#define DHT_PIN 4
#define DHTTYPE DHT22

#define WATER_TEMP_PIN 5

#define EC_PIN 34

#define PH_SENSOR_PIN 35

#define TRIG_PIN 18
#define ECHO_PIN 19

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

constexpr unsigned long MIXING_DURATION =
    60UL * 1000UL;
constexpr unsigned long PH_DOSE_COOLDOWN =
    60000UL;
constexpr unsigned long EC_DOSE_COOLDOWN =
    60000UL;

// ======================================================

constexpr unsigned long SENSOR_STABILIZATION_TIME = 10000UL;

constexpr unsigned long STARTUP_ON_TIME = 10000UL;

constexpr unsigned long STARTUP_OFF_TIME = 5000UL;

constexpr unsigned long NORMAL_FOG_ON_TIME = 5000UL;

constexpr unsigned long NORMAL_FOG_OFF_TIME = 5000UL;

constexpr unsigned long HOT_FOG_ON_TIME =
    5UL * 60UL * 1000UL;

constexpr unsigned long HOT_FOG_OFF_TIME =
    2UL * 60UL * 1000UL;

constexpr unsigned long COLD_FOG_ON_TIME =
    2UL * 60UL * 1000UL;

constexpr unsigned long COLD_FOG_OFF_TIME =
    5UL * 60UL * 1000UL;
// ======================================================

constexpr int OUT_OF_RANGE_REQUIRED = 3;

// ======================================================
// Debug
// ======================================================

constexpr bool DEBUG_ENABLED = true;
constexpr unsigned long DEBUG_INTERVAL = 1000UL;

// ======================================================
// Water Refill
// ======================================================

constexpr float REFILL_START_LEVEL = 20.0f;

constexpr float REFILL_STOP_LEVEL = 75.0f;

#endif