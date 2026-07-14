#include "DebugManager.h"
#include "ActuatorManager.h"

#include "Globals.h"
#include "Config.h"

extern ActuatorManager actuatorManager;

void DebugManager::begin()
{
    lastPrintTime = 0;

    showSensorPage = true;
}

void DebugManager::update()
{
    if (!DEBUG_ENABLED)
        return;

    if (millis() - lastPrintTime < DEBUG_INTERVAL)
        return;

    lastPrintTime = millis();

    if (showSensorPage)
    {
        printSensors();
    }
    else
    {
        printActuators();
    }

    showSensorPage = !showSensorPage;
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

    Serial.print("Current Mode : ");

    switch (systemState.currentMode)
    {
        case STARTUP:
            Serial.println("STARTUP");
            break;

        case NORMAL:
            Serial.println("NORMAL");
            break;

        case REFILLING:
            Serial.println("REFILLING");
            break;

        case DOSING_PH:
            Serial.println("DOSING_PH");
            break;

        case STABILIZING_PH:
            Serial.println("STABILIZING_PH");
            break;

        case DOSING_EC:
            Serial.println("DOSING_EC");
            break;

        case STABILIZING_EC:
            Serial.println("STABILIZING_EC");
            break;

        case SAFETY_LOCK:
            Serial.println("SAFETY_LOCK");
            break;
    }

    Serial.println();

    printFloat(
        "Air Temperature",
        sensors.temperature,
        "C",
        2);

    printFloat(
        "Humidity",
        sensors.humidity,
        "%",
        2);

    printFloat(
        "Water Temperature",
        sensors.waterTemp,
        "C",
        2);

    printFloat(
        "Water Level",
        sensors.waterLevel,
        "%",
        1);

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

void DebugManager::printActuators()
{
    printHeader("ACTUATOR STATES");

    printBool(
        "Fogger",
        actuatorManager.isOn(FOGGER));

    printBool(
        "Grow Light",
        actuatorManager.isOn(GROW_LIGHT));

    printBool(
        "Blower",
        actuatorManager.isOn(BLOWER));

    printBool(
        "Solenoid",
        actuatorManager.isOn(SOLENOID));

    printBool(
        "Grow Pump",
        actuatorManager.isOn(GROW_PUMP));

    printBool(
        "Bloom Pump",
        actuatorManager.isOn(BLOOM_PUMP));

    printBool(
        "pH Up Pump",
        actuatorManager.isOn(PH_UP_PUMP));

    printBool(
        "pH Down Pump",
        actuatorManager.isOn(PH_DOWN_PUMP));

    printBool(
        "Water Heater",
        actuatorManager.isOn(WATER_HEATER));

    printBool(
        "Peltier",
        actuatorManager.isOn(PELTIER));

    printSeparator();
}