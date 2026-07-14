#include "DebugManager.h"

#include "Globals.h"
#include "Config.h"

void DebugManager::begin()
{
    lastPrintTime = 0;
}

void DebugManager::update()
{
    if (!DEBUG_ENABLED)
        return;

    if (millis() - lastPrintTime < DEBUG_INTERVAL)
        return;

    lastPrintTime = millis();

    printSensors();
}

void DebugManager::printHeader(const char *title)
{
    Serial.println();
    Serial.println("========================================");
    Serial.print(" BASILIENCE - ");
    Serial.println(title);
    Serial.println("========================================");
}

void DebugManager::printSeparator()
{
    Serial.println("----------------------------------------");
}

void DebugManager::printFloat(
    const char *label,
    float value,
    const char *unit,
    uint8_t decimals)
{
    Serial.print(label);

    Serial.print(" : ");

    Serial.print(value, decimals);

    if (unit != nullptr)
    {
        Serial.print(" ");

        Serial.print(unit);
    }

    Serial.println();
}

void DebugManager::printInteger(
    const char *label,
    int value,
    const char *unit)
{
    Serial.print(label);

    Serial.print(" : ");

    Serial.print(value);

    if (unit != nullptr)
    {
        Serial.print(" ");

        Serial.print(unit);
    }

    Serial.println();
}

void DebugManager::printBool(
    const char *label,
    bool value)
{
    Serial.print(label);

    Serial.print(" : ");

    Serial.println(value ? "ON" : "OFF");
}

void DebugManager::printSensors()
{
    printHeader("SENSOR DATA");

    printFloat(
        "Air Temperature",
        sensors.temperature,
        "°C",
        2);

    printFloat(
        "Humidity",
        sensors.humidity,
        "%",
        2);

    printFloat(
        "Water Temperature",
        sensors.waterTemp,
        "°C",
        2);

    printFloat(
        "Water Level",
        sensors.waterLevel,
        "%",
        1);

    printFloat(
        "EC",
        sensors.ec,
        "mS/cm",
        3);

    printInteger(
        "EC ADC",
        sensors.ecRaw,
        nullptr);

    printFloat(
        "EC Voltage",
        sensors.ecVoltage,
        "V",
        3);

    printFloat(
        "EC",
        sensors.ec,
        "mS/cm",
        3);

    printFloat(
        "TDS",
        sensors.tds,
        "ppm",
        0);

    printInteger(
        "pH mV",
        sensors.phMilliVolts,
        "mV");

    printFloat(
        "pH",
        sensors.ph,
        nullptr,
        2);

    printSeparator();
}