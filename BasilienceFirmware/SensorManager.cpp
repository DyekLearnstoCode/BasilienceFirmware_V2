#include "SensorManager.h"
#include "Config.h"
#include "Globals.h"

#include <cstring>

SensorManager::SensorManager()
    : dht(DHT_PIN, DHTTYPE),
      oneWire(WATER_TEMP_PIN),
      waterSensor(&oneWire)
{
    ecSampleIndex = 0;

    ecBufferFilled = false;

    lastECSample = 0;

    for (int i = 0; i < EC_SAMPLE_COUNT; i++)
    {
        ecSamples[i] = 0;
    }
}

float SensorManager::measureDistanceCM()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0)
        return -1;

    return duration * 0.0343f / 2.0f;
}

void SensorManager::updateECSamples()
{

    if (millis() - lastECSample < EC_SAMPLE_INTERVAL)
        return;

    lastECSample = millis();

    ecSamples[ecSampleIndex] = analogRead(EC_PIN);

    ecSampleIndex++;

    if (ecSampleIndex >= EC_SAMPLE_COUNT)
    {
        ecSampleIndex = 0;

        ecBufferFilled = true;
    }
}

int SensorManager::getMedianADC()
{
    if (!ecBufferFilled)
        return 0;

    int sorted[EC_SAMPLE_COUNT];

    memcpy(
        sorted,
        ecSamples,
        sizeof(ecSamples));

    for (int i = 0; i < EC_SAMPLE_COUNT - 1; i++)
    {
        for (int j = i + 1; j < EC_SAMPLE_COUNT; j++)
        {
            if (sorted[j] < sorted[i])
            {
                int t = sorted[i];

                sorted[i] = sorted[j];

                sorted[j] = t;
            }
        }
    }

    return sorted[EC_SAMPLE_COUNT / 2];
}

void SensorManager::begin()
{
    dht.begin();

    waterSensor.begin();
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void SensorManager::update()
{
    updateECSamples();

    readDHT();

    readWaterTemperature();

    readEC();

    readPH();

    readWaterLevel();
}

void SensorManager::readDHT()
{
    float humidity = dht.readHumidity();

    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature))
        return;

    sensors.humidity = humidity;

    sensors.temperature = temperature;
}

void SensorManager::readWaterTemperature()
{
    waterSensor.requestTemperatures();

    float temp = waterSensor.getTempCByIndex(0);

    if (temp == DEVICE_DISCONNECTED_C)
        return;

    sensors.waterTemp = temp;
}

void SensorManager::readEC()
{
    if (!ecBufferFilled)
        return;

    float adc = getMedianADC();

    sensors.ecRaw = (int)adc;

    float voltage =
        (adc * ADC_REFERENCE) /
        ADC_RESOLUTION;

    sensors.ecVoltage = voltage;

    float compensationCoefficient =
        1.0f +
        0.02f *
            (sensors.waterTemp - 25.0f);

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

    sensors.ec = ec;

    sensors.tds = ec * 500.0f;
}
void SensorManager::readPH()
{
}

void SensorManager::readWaterLevel()
{
    float distance = measureDistanceCM();

    if (distance < 0)
        return;

    constexpr float EMPTY_DISTANCE = 30.0f;

    constexpr float FULL_DISTANCE = 5.0f;

    sensors.waterLevel = constrain(
        (EMPTY_DISTANCE - distance) /
            (EMPTY_DISTANCE - FULL_DISTANCE) * 100.0f,
        0.0f,
        100.0f);
}