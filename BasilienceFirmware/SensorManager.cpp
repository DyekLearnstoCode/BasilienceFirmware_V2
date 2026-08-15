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

    analogReadResolution(12);

    analogSetAttenuation(ADC_11db);

    pinMode(TRIG_PIN, OUTPUT);

    pinMode(ECHO_PIN, INPUT);

    ecSampler.begin();

    phSampler.begin();
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

void SensorManager::applyEffectiveSensors()
{
    // Mock mode's enabled/disabled state lives only in Firebase and is not
    // restored locally at boot (systemState.mockSensorsEnabled defaults to
    // false), so right after a reboot/brownout we don't yet know whether a
    // previously-enabled mock session should still own control. Until
    // FirebaseManager confirms the source at least once, hold every
    // effective reading explicitly invalid instead of defaulting to physical
    // - a plausible-looking physical reading at this point (e.g. an
    // unsettled water level) must never trigger automation/alerts.
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

    waterSensor.requestTemperatures();

    float temp =
        waterSensor.getTempCByIndex(0);

    if (temp == DEVICE_DISCONNECTED_C || !isfinite(temp))
    {
        if (waterTempFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            waterTempFailureStreak++;

            if (waterTempFailureStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
            {
                Serial.print("[SENSOR] Water temperature transient read failure ");
                Serial.print(waterTempFailureStreak);
                Serial.print("/");
                Serial.println(SENSOR_TRANSIENT_FAILURE_THRESHOLD);
            }
            else
            {
                Serial.println("[SENSOR] Water temperature confirmed unavailable");
                physicalSensors.waterTemp = NAN;
            }
        }
        // Already confirmed unavailable - stays NaN, no repeated logging.
        return;
    }

    if (waterTempFailureStreak >= SENSOR_TRANSIENT_FAILURE_THRESHOLD)
    {
        Serial.println("[SENSOR] Water temperature recovered");
    }

    waterTempFailureStreak = 0;
    lastValidWaterTemp = temp;
    physicalSensors.waterTemp = temp;
}

void SensorManager::readWaterLevel()
{
    float distance =
        measureDistanceCM();

    if (distance < 0 || !isfinite(distance))
    {
        physicalSensors.waterLevel = NAN;
        physicalSensors.waterLevelDistanceCm = NAN;
        return;
    }

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
