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
    // Automation, alerts, safety, and Firebase publication all consume this one
    // effective dataset. Physical sampling remains active in physicalSensors.
    sensors = physicalSensors;

    if (systemState.mockSensorsEnabled)
    {
        if (!isnan(systemState.mockSensors.temperature)) sensors.temperature = systemState.mockSensors.temperature;
        if (!isnan(systemState.mockSensors.humidity))    sensors.humidity = systemState.mockSensors.humidity;
        if (!isnan(systemState.mockSensors.waterTemp))   sensors.waterTemp = systemState.mockSensors.waterTemp;
        if (!isnan(systemState.mockSensors.waterLevel))  sensors.waterLevel = systemState.mockSensors.waterLevel;
        if (!isnan(systemState.mockSensors.ph))          sensors.ph = systemState.mockSensors.ph;
        if (!isnan(systemState.mockSensors.ec))
        {
            sensors.ec = systemState.mockSensors.ec;
            sensors.tds = sensors.ec * 500.0f;
        }

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
    else if (systemState.mockApplyPending)
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
    waterSensor.requestTemperatures();

    float temp =
        waterSensor.getTempCByIndex(0);

    if (temp == DEVICE_DISCONNECTED_C || !isfinite(temp))
    {
        physicalSensors.waterTemp = NAN;
        return;
    }

    physicalSensors.waterTemp =
        temp;
}

void SensorManager::readWaterLevel()
{
    float distance =
        measureDistanceCM();

    if (distance < 0 || !isfinite(distance))
    {
        physicalSensors.waterLevel = NAN;
        return;
    }

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

    float compensationCoefficient =
        1.0f +
        0.02f *
            (physicalSensors.waterTemp - 25.0f);

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
