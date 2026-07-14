#include "SensorManager.h"

#include "Globals.h"
#include "Config.h"

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

    ecSampler.begin();

    phSampler.begin();

    analogReadResolution(12);

    analogSetAttenuation(ADC_11db);

    pinMode(TRIG_PIN, OUTPUT);

    pinMode(ECHO_PIN, INPUT);
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
        return;

    sensors.humidity =
        humidity;

    sensors.temperature =
        temperature;
}

void SensorManager::readWaterTemperature()
{
    waterSensor.requestTemperatures();

    float temp =
        waterSensor.getTempCByIndex(0);

    if (temp == DEVICE_DISCONNECTED_C)
        return;

    sensors.waterTemp =
        temp;
}

void SensorManager::readWaterLevel()
{
    float distance =
        measureDistanceCM();

    if (distance < 0)
        return;

    constexpr float EMPTY_DISTANCE = 30.0f;

    constexpr float FULL_DISTANCE = 5.0f;

    sensors.waterLevel =
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
    if (!phSampler.ready())
        return;

    int millivolts =
        phSampler.average();

    sensors.phMilliVolts =
        millivolts;

    sensors.ph =
        PH_SLOPE *
            millivolts +
        PH_OFFSET;
}