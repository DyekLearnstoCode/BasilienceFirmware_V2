// ============================================================
// DHT22_Diagnostic.ino
//
// TEMPORARY standalone diagnostic sketch. NOT part of the Basilience
// production firmware and must never be merged into it.
//
// Purpose: determine whether the physical DHT22 <-> ESP32 communication
// itself is reliable in isolation, with every other subsystem (WiFi,
// Firebase, GSM, other sensors, actuators) completely absent, so any
// intermittent nan streak observed here can only be explained by the
// sensor/wiring/library/timing - not by anything Basilience-specific.
//
// Hardware:
//   ESP32, DHT22 DATA on GPIO4, powered exactly as currently wired.
// Library:
//   Adafruit "DHT sensor library" v1.4.7 (classic DHT.h API).
// ============================================================

#include <DHT.h>

// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

constexpr uint8_t DHT_PIN = 4;
constexpr uint8_t DHT_TYPE = DHT22;

constexpr unsigned long READ_INTERVAL_MS = 2500UL;
constexpr unsigned long STATS_INTERVAL_MS = 30000UL;

DHT dht(DHT_PIN, DHT_TYPE);

// ------------------------------------------------------------
// Scheduling state (millis()-based, no delay() in the read/report loop)
// ------------------------------------------------------------

unsigned long lastReadAt = 0;
unsigned long lastStatsAt = 0;

// ------------------------------------------------------------
// Counters - raw, undebounced. No smoothing, no retries, no NaN
// substitution: every acquisition is reported exactly as the library
// returned it.
// ------------------------------------------------------------

unsigned long totalReads = 0;
unsigned long validReads = 0;
unsigned long failedReads = 0;
unsigned long currentStreak = 0;
unsigned long maxStreak = 0;

void setup()
{
    Serial.begin(115200);

    dht.begin();

    Serial.println("[DHT-ONLY] Standalone DHT22 diagnostic starting");
    Serial.print("[DHT-ONLY] pin=");
    Serial.print(DHT_PIN);
    Serial.print(" type=DHT22 readIntervalMs=");
    Serial.print(READ_INTERVAL_MS);
    Serial.print(" statsIntervalMs=");
    Serial.println(STATS_INTERVAL_MS);

    // First read fires READ_INTERVAL_MS after this point (loop()'s own
    // millis()-based check below), which also naturally covers the DHT22's
    // own post-power-up settling time - no extra delay() needed here.
    lastReadAt = millis();
    lastStatsAt = millis();
}

void loop()
{
    const unsigned long now = millis();

    if (now - lastReadAt >= READ_INTERVAL_MS)
    {
        lastReadAt = now;

        // Normal Adafruit DHT API usage: readHumidity() then
        // readTemperature(), exactly as the library recommends.
        const float humidity = dht.readHumidity();
        const float temperature = dht.readTemperature();

        const bool valid = isfinite(humidity) && isfinite(temperature);

        totalReads++;

        if (valid)
        {
            validReads++;
            currentStreak = 0;
        }
        else
        {
            failedReads++;
            currentStreak++;
            if (currentStreak > maxStreak)
            {
                maxStreak = currentStreak;
            }
        }

        Serial.print("[DHT-ONLY] t=");
        Serial.print(now);
        Serial.print(" temp=");
        Serial.print(temperature, 2);
        Serial.print(" humidity=");
        Serial.print(humidity, 2);
        Serial.print(" valid=");
        Serial.println(valid ? "true" : "false");
    }

    if (now - lastStatsAt >= STATS_INTERVAL_MS)
    {
        lastStatsAt = now;

        const float failRate = totalReads > 0
            ? (100.0f * (float)failedReads / (float)totalReads)
            : 0.0f;

        Serial.print("[DHT-STATS] total=");
        Serial.print(totalReads);
        Serial.print(" valid=");
        Serial.print(validReads);
        Serial.print(" failed=");
        Serial.print(failedReads);
        Serial.print(" failRate=");
        Serial.print(failRate, 1);
        Serial.print("% currentStreak=");
        Serial.print(currentStreak);
        Serial.print(" maxStreak=");
        Serial.println(maxStreak);
    }
}
