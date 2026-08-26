#include "SensorManager.h"

#include "Globals.h"
#include "Config.h"
#include "Calibration.h"

SensorManager::SensorManager()

    :

      dht(DHT_PIN, DHTTYPE),

      oneWire(WATER_TEMP_PIN),

      waterSensor(&oneWire),

      ecSampler(
          EC_PIN,
          EC_SAMPLE_COUNT,
          EC_SAMPLE_INTERVAL,
          AnalogSampler::RAW_ADC),

      phSampler(
          PH_SENSOR_PIN,
          PH_SAMPLE_COUNT,
          PH_SAMPLE_INTERVAL,
          AnalogSampler::MILLIVOLTS)

{
}

void SensorManager::begin()
{
    dht.begin();

    waterSensor.begin();

    Serial.print("[DS18B20] GPIO: ");
    Serial.println(WATER_TEMP_PIN);

    waterSensorDeviceCount = waterSensor.getDeviceCount();
    Serial.print("[DS18B20] Devices found: ");
    Serial.println(waterSensorDeviceCount);

    if (waterSensorDeviceCount > 0)
    {
        waterSensorAddressValid = waterSensor.getAddress(waterSensorAddress, 0);
        if (!waterSensorAddressValid)
        {
            Serial.println("[DS18B20] Device present but address could not be retrieved - falling back to index-based read");
        }
    }
    // A count of 0 here is not fatal - readWaterTemperature() re-enumerates
    // on its own throttled cadence and recovers automatically if the probe
    // wasn't settled yet at this point in boot (see its own comment).

    analogReadResolution(12);

    analogSetAttenuation(ADC_11db);

    pinMode(TRIG_PIN, OUTPUT);

    pinMode(ECHO_PIN, INPUT);

    ecSampler.begin();

    phSampler.begin();

    // SensorData's default waterLevel=0 is deliberately a valid real-world
    // reading (empty tank), unlike every other field here which defaults to
    // NaN - so it's the one field where "never sampled yet" and "genuinely
    // measured empty" are otherwise indistinguishable. Before readWaterLevel()
    // has ever completed a successful measurement (or reached its own
    // confirmed-unavailable threshold), physicalSensors.waterLevel must not
    // read as a plausible 0% to validWaterLevel()/isfinite() and unlock
    // refill/fog/dosing/cooling on a placeholder. This does not touch what a
    // REAL measured 0% means afterward - readWaterLevel()'s success path
    // unconditionally overwrites this with the actual computed percentage.
    physicalSensors.waterLevel = NAN;

    resolveLocalSensorSource();
}

// Decides the effective sensor source locally, at boot, without any network.
//
// Firebase used to be the only thing that could ever set sensorSourceResolved,
// so a unit that cold-booted with no Wi-Fi held every effective reading at NaN
// forever and local automation never engaged. The mock flag is now persisted
// in NVS, which means the same integrity guarantee (a previously-enabled mock
// session must not be silently replaced by physical readings after a reboot)
// can be honoured from local storage instead of from the cloud.
//
// PHYSICAL is the safe default: an unknown or never-written flag resolves to
// real sensors, never to simulated ones.
void SensorManager::resolveLocalSensorSource()
{
    bool mockEnabled = false;

    if (sourcePreferences.begin(SOURCE_NVS_NAMESPACE, true))
    {
        mockEnabled = sourcePreferences.getBool(SOURCE_NVS_KEY, false);
        sourcePreferences.end();
    }

    systemState.mockSensorsEnabled = mockEnabled;
    systemState.sensorSourceResolved = true;

    // Prime the change-detection used by applyEffectiveSensors() so the source
    // is announced exactly once here rather than again on the first update().
    sensorSourceReported = true;
    lastReportedMockSource = mockEnabled;

    if (mockEnabled)
    {
        // Mock readings are never persisted, so a mock session that survives a
        // reboot starts with no values, and physical readings must never
        // backfill mock mode. Rather than idle forever if the payload never
        // arrives, arm a bounded wait that reverts to physical sensors.
        mockBootWaitingForPayload = true;
        mockBootWaitStartedAt = millis();

        Serial.println("[AUTOMATION] Sensor source=MOCK (persisted)");
        Serial.println("[AUTOMATION] Waiting for fresh mock payload...");
    }
    else
    {
        Serial.println("[AUTOMATION] Sensor source=PHYSICAL (local)");
    }
}

void SensorManager::persistSensorSource(bool mockEnabled)
{
    if (!sourcePreferences.begin(SOURCE_NVS_NAMESPACE, false)) return;

    // Write only on a real change - this runs from the periodic mock read.
    if (sourcePreferences.getBool(SOURCE_NVS_KEY, false) != mockEnabled)
    {
        sourcePreferences.putBool(SOURCE_NVS_KEY, mockEnabled);
    }
    sourcePreferences.end();
}

void SensorManager::update()
{
    ecSampler.update();

    phSampler.update();

    readDHT();

    readWaterTemperature();

    readWaterLevel();

    readEC();

    readPH();

    applyEffectiveSensors();
}

// Bounded recovery for a boot-restored mock source. Runs every tick and is
// independent of connectivity, so it still fires with no Wi-Fi at all.
void SensorManager::updateMockBootWait()
{
    if (!mockBootWaitingForPayload) return;

    // Unsigned subtraction - safe across the millis() rollover.
    if (millis() - mockBootWaitStartedAt < MOCK_BOOT_PAYLOAD_TIMEOUT) return;

    mockBootWaitingForPayload = false;

    systemState.mockSensorsEnabled = false;
    persistSensorSource(false);

    // Drop the empty mock dataset so nothing stale can be read back if mock
    // mode is later re-enabled before a payload arrives.
    systemState.mockSensors = SensorData();
    systemState.mockSensors.waterLevel = NAN;

    Serial.println("[AUTOMATION] Mock payload timeout - reverting to PHYSICAL sensors");
}

void SensorManager::notifyMockPayloadReceived()
{
    if (!mockBootWaitingForPayload) return;

    mockBootWaitingForPayload = false;
    Serial.println("[AUTOMATION] Fresh mock payload received - remaining in MOCK mode");
}

void SensorManager::cancelMockBootWait()
{
    mockBootWaitingForPayload = false;
}

void SensorManager::applyEffectiveSensors()
{
    // Evaluated before the source is selected below, so the tick that times
    // out already publishes physical readings rather than waiting one more.
    updateMockBootWait();

    // Mock mode's enabled/disabled state lives only in Firebase and is not
    // restored locally at boot (systemState.mockSensorsEnabled defaults to
    // false), so right after a reboot/brownout we don't yet know whether a
    // previously-enabled mock session should still own control. Until
    // FirebaseManager confirms the source at least once, hold every
    // effective reading explicitly invalid instead of defaulting to physical
    // - a plausible-looking physical reading at this point (e.g. an
    // unsettled water level) must never trigger automation/alerts.
    // Defensive only. resolveLocalSensorSource() resolves the source during
    // begin(), before the first update(), so this no longer gates a cold boot
    // on Firebase; it remains as a guard against any future path that clears
    // the flag.
    if (!systemState.sensorSourceResolved)
    {
        sensors = SensorData();
        sensors.waterLevel = NAN;

        if (!sensorSourceWaitingLogged)
        {
            Serial.println("[AUTOMATION] Sensor source=WAITING");
            sensorSourceWaitingLogged = true;
        }
        return;
    }

    // A persisted-MOCK boot has no mock readings of its own yet - mock values
    // are never persisted, so systemState.mockSensors is still the plain
    // default-constructed SensorData() at this point. That default is a
    // legitimate "no data" placeholder everywhere except waterLevel, which
    // defaults to 0 because 0 IS a valid real-sensor reading (empty tank -
    // see SensorData's own comment). Left alone here, that placeholder 0
    // reads as a genuine "tank empty" to every isfinite()-based safety check
    // (validWaterLevel() et al.), which is exactly what let automatic REFILL
    // fire the solenoid off boot-restored mock state before any payload had
    // ever arrived. Hold every effective field explicitly invalid - same
    // technique as the sensorSourceResolved guard above - until either a
    // fresh, validated payload arrives (notifyMockPayloadReceived(), which
    // FirebaseManager::readMockSensors() only calls after pH/EC and the rest
    // have all parsed successfully) or updateMockBootWait() above reverts to
    // PHYSICAL after its own timeout. Local safety shutdowns are untouched:
    // they command actuators OFF unconditionally and never gate on `sensors`
    // validity, so this only withholds permission to act, never the ability
    // to stand down. RTC/cycle restoration and grow-light scheduling are
    // unaffected too - grow light is driven purely by RTC time, not by any
    // field in `sensors`.
    if (systemState.mockSensorsEnabled && mockBootWaitingForPayload)
    {
        sensors = SensorData();
        sensors.waterLevel = NAN;

        if (!mockBootWaitHeldLogged)
        {
            Serial.println("[AUTOMATION] Mock boot wait - holding automation safe until fresh payload arrives");
            mockBootWaitHeldLogged = true;
        }
        return;
    }
    mockBootWaitHeldLogged = false;

    // Automation, alerts, safety, and Firebase publication all consume this one
    // effective dataset. Physical sampling remains active in physicalSensors
    // regardless of mock mode (Developer Sensor Test reads it directly), but
    // it must never backfill a field the mock command left unset. Mock mode
    // is meant to be a fully controlled simulated environment - a field the
    // mock payload didn't include stays invalid (NaN) rather than silently
    // reverting to a real, possibly noisy physical reading.
    if (systemState.mockSensorsEnabled)
    {
        sensors = systemState.mockSensors;
        sensors.tds = isnan(sensors.ec) ? NAN : sensors.ec * 500.0f;

        if (systemState.mockApplyPending)
        {
            Serial.println("[MOCK] Applied to effective firmware SensorData");
            Serial.print("[MOCK] Effective pH=");
            Serial.println(sensors.ph, 2);
            Serial.print("[MOCK] Effective EC=");
            Serial.println(sensors.ec, 2);
            systemState.mockApplyPending = false;
        }
    }
    else
    {
        sensors = physicalSensors;

        if (systemState.mockApplyPending)
        {
            Serial.println("[MOCK] Mock sensor mode DISABLED");
            Serial.println("[MOCK] Physical sensors restored as automation source");
            Serial.print("[PHYSICAL] pH="); Serial.println(sensors.ph, 2);
            Serial.print("[PHYSICAL] EC="); Serial.println(sensors.ec, 2);
            Serial.print("[PHYSICAL] AirTemp="); Serial.println(sensors.temperature, 2);
            Serial.print("[PHYSICAL] Humidity="); Serial.println(sensors.humidity, 2);
            Serial.print("[PHYSICAL] WaterTemp="); Serial.println(sensors.waterTemp, 2);
            Serial.print("[PHYSICAL] WaterLevel="); Serial.println(sensors.waterLevel, 2);
            systemState.mockApplyPending = false;
        }
    }

    if (!sensorSourceReported || lastReportedMockSource != systemState.mockSensorsEnabled)
    {
        Serial.print("[AUTOMATION] Sensor source=");
        Serial.println(systemState.mockSensorsEnabled ? "MOCK" : "PHYSICAL");
        sensorSourceReported = true;
        lastReportedMockSource = systemState.mockSensorsEnabled;
    }
}

float SensorManager::measureDistanceCM()
{
    digitalWrite(TRIG_PIN, LOW);

    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);

    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    long duration =
        pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0)
        return -1;

    return duration * 0.0343f / 2.0f;
}

void SensorManager::readDHT()
{
    float humidity =
        dht.readHumidity();

    float temperature =
        dht.readTemperature();

    if (isnan(humidity) || isnan(temperature))
    {
        physicalSensors.humidity = NAN;
        physicalSensors.temperature = NAN;
        return;
    }

    physicalSensors.humidity =
        humidity;

    physicalSensors.temperature =
        temperature;
}

void SensorManager::readWaterTemperature()
{
    // The DS18B20 conversion is a blocking OneWire transaction; running it
    // every loop iteration both stalls loop() and increases how often it can
    // collide with other blocking work (e.g. Firebase calls). Not due yet
    // simply means physicalSensors.waterTemp keeps its last value - it must
    // never be invalidated merely because a new read isn't scheduled.
    if (millis() - lastWaterTempReadTime < WATER_TEMP_READ_INTERVAL_MS)
    {
        return;
    }
    lastWaterTempReadTime = millis();

    if (waterSensorDeviceCount == 0)
    {
        // Re-enumerate on the same throttled cadence as the read itself - no
        // new timer, no delay(). Recovers automatically if the probe wasn't
        // settled/responding yet at begin() (e.g. long cable run, power-up
        // settling) and starts answering later.
        waterSensor.begin();
        waterSensorDeviceCount = waterSensor.getDeviceCount();

        if (waterSensorDeviceCount > 0)
        {
            Serial.print("[DS18B20] Device found on retry - count: ");
            Serial.println(waterSensorDeviceCount);
            waterSensorAddressValid = waterSensor.getAddress(waterSensorAddress, 0);
        }
    }

    float temp = NAN;
    if (waterSensorDeviceCount > 0)
    {
        waterSensor.requestTemperatures();
        temp = waterSensorAddressValid
            ? waterSensor.getTempC(waterSensorAddress)
            : waterSensor.getTempCByIndex(0);
    }

    // DS18B20 commonly returns exactly 85.00C as a power-on/default
    // conversion result rather than a real reading (its scratchpad reset
    // value) - a small float tolerance avoids an unsafe exact comparison
    // while still only catching that specific default, not a genuine
    // ~85C reading (implausible for a hydroponic reservoir regardless).
    const bool powerOnDefault = fabsf(temp - 85.0f) < 0.01f;

    if (waterSensorDeviceCount == 0)
    {
        Serial.println("[DS18B20] Raw: no device enumerated");
    }
    else if (temp == DEVICE_DISCONNECTED_C)
    {
        Serial.println("[DS18B20] Raw: -127.00 C (DEVICE_DISCONNECTED_C)");
    }
    else if (powerOnDefault)
    {
        Serial.print("[DS18B20] Raw: "); Serial.print(temp, 2); Serial.println(" C (power-on/default)");
    }
    else if (!isfinite(temp))
    {
        Serial.println("[DS18B20] Raw: invalid (non-finite)");
    }
    else
    {
        Serial.print("[DS18B20] Raw: "); Serial.print(temp, 2); Serial.println(" C");
    }

    const bool invalid =
        waterSensorDeviceCount == 0 ||
        temp == DEVICE_DISCONNECTED_C ||
        !isfinite(temp) ||
        powerOnDefault;

    if (invalid)
    {
        if (waterTempFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            waterTempFailureStreak++;

            if (waterTempFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
            {
                Serial.print("[DS18B20] transient failure ");
                Serial.print(waterTempFailureStreak);
                Serial.print("/");
                Serial.println(SENSOR_TRANSIENT_FAILURE_THRESHOLD);
            }
            else
            {
                Serial.println("[DS18B20] confirmed unavailable");
                physicalSensors.waterTemp = NAN;
            }
        }
        // Already confirmed unavailable - stays NaN, no repeated logging.
        return;
    }

    if (waterTempFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD)
    {
        Serial.print("[DS18B20] recovered: ");
        Serial.print(temp, 2);
        Serial.println(" C");
    }

    waterTempFailureStreak = 0;
    lastValidWaterTemp = temp;
    physicalSensors.waterTemp = temp;
}

void SensorManager::readWaterLevel()
{
    // The HC-SR04 trigger/echo cycle needs real settling time; re-triggering
    // on every loop iteration is a common cause of spurious pulseIn()
    // timeouts unrelated to the sensor or wiring actually failing. Not due
    // yet simply means the last values are kept - they must never be
    // invalidated merely because a new read isn't scheduled.
    if (millis() - lastWaterLevelReadTime < WATER_LEVEL_READ_INTERVAL_MS)
    {
        return;
    }
    lastWaterLevelReadTime = millis();

    float distance =
        measureDistanceCM();

    if (distance < 0 || !isfinite(distance))
    {
        if (waterLevelFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            waterLevelFailureStreak++;

            if (waterLevelFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
            {
                Serial.print("[SENSOR] Water level transient read failure ");
                Serial.print(waterLevelFailureStreak);
                Serial.print("/");
                Serial.println(SENSOR_TRANSIENT_FAILURE_THRESHOLD);
            }
            else
            {
                Serial.println("[SENSOR] Water level confirmed unavailable");
                physicalSensors.waterLevel = NAN;
                physicalSensors.waterLevelDistanceCm = NAN;
            }
        }
        // Already confirmed unavailable - stays NaN, no repeated logging.
        return;
    }

    if (waterLevelFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD)
    {
        Serial.println("[SENSOR] Water level recovered");
    }
    waterLevelFailureStreak = 0;

    physicalSensors.waterLevelDistanceCm = distance;

    constexpr float EMPTY_DISTANCE = 30.0f;

    constexpr float FULL_DISTANCE = 5.0f;

    physicalSensors.waterLevel =
        constrain(

            (EMPTY_DISTANCE - distance)

                /

                (EMPTY_DISTANCE - FULL_DISTANCE)

                * 100.0f,

            0.0f,

            100.0f);
}

void SensorManager::readEC()
{
    if (!ecSampler.ready())
        return;

    float adc = ecSampler.median();

    physicalSensors.ecRaw = (int)adc;

    float voltage =
        (adc * ADC_REFERENCE) /
        ADC_RESOLUTION;

    physicalSensors.ecVoltage = voltage;

    // A NaN water temperature must never reach the compensation formula - it
    // would make EC itself go NaN even though the EC sensor is fine. Fall
    // back to the last known-good reading, and only to a fixed default if
    // none has ever been captured this session.
    float compensationTemp;
    EcCompensationSource compensationSource;

    if (isfinite(physicalSensors.waterTemp))
    {
        compensationTemp = physicalSensors.waterTemp;
        compensationSource = EcCompensationSource::LIVE;
    }
    else if (isfinite(lastValidWaterTemp))
    {
        compensationTemp = lastValidWaterTemp;
        compensationSource = EcCompensationSource::LAST_VALID;
    }
    else
    {
        compensationTemp = 25.0f;
        compensationSource = EcCompensationSource::FALLBACK_DEFAULT;
    }

    if (compensationSource != lastEcCompensationSource)
    {
        if (compensationSource == EcCompensationSource::LAST_VALID)
        {
            Serial.println("[SENSOR] EC compensation using last valid water temperature");
        }
        else if (compensationSource == EcCompensationSource::FALLBACK_DEFAULT)
        {
            Serial.println("[SENSOR] EC compensation using 25C fallback");
        }
        lastEcCompensationSource = compensationSource;
    }

    float compensationCoefficient =
        1.0f +
        0.02f *
            (compensationTemp - 25.0f);

    float compensationVoltage =
        voltage /
        compensationCoefficient;

    float tds =
        (133.42f * compensationVoltage * compensationVoltage * compensationVoltage -
         255.86f * compensationVoltage * compensationVoltage +
         857.39f * compensationVoltage) *
        0.5f;

    float ec =
        (tds / 500.0f) *
        EC_FACTOR;

    physicalSensors.ec = ec;

    physicalSensors.tds = ec * 500.0f;
}

void SensorManager::readPH()
{
    if (!phSampler.ready())
        return;

    int millivolts =
        phSampler.average();

    physicalSensors.phMilliVolts =
        millivolts;

    physicalSensors.ph =
        PH_SLOPE *
            millivolts +
        PH_OFFSET;
}
