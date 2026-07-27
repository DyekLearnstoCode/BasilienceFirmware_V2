#include "DebugManager.h"
#include "ActuatorManager.h"

#include "Globals.h"
#include "Config.h"
#include "RTCManager.h"

void DebugManager::begin()
{
    lastPrintTime = 0;

    currentPage = 0;
}

void DebugManager::update()
{
    if (!DEBUG_ENABLED)
        return;

    if (millis() - lastPrintTime < DEBUG_INTERVAL)
        return;

    lastPrintTime = millis();

    switch (currentPage)
    {
        case 0:
            printSystemStatus();
            break;

        case 1:
            printSensors();
            break;

        case 2:
            printAlerts();
            break;

        case 3:
            printActuators();
            break;

        case 4:
            printRTC();
            break;
    }

    currentPage++;

    if (currentPage > 4)
        currentPage = 0;
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

const char* DebugManager::getModeName(
    SystemMode mode)
{
    switch(mode)
    {
        case SENSOR_STABILIZATION:
            return "SENSOR_STABILIZATION";

        case STARTUP:
            return "STARTUP";

        case NORMAL:
            return "NORMAL";

        case REFILLING:
            return "REFILLING";

        case DOSING_PH:
            return "DOSING_PH";

        case STABILIZING_PH:
            return "STABILIZING_PH";

        case DOSING_EC:
            return "DOSING_EC";

        case STABILIZING_EC:
            return "STABILIZING_EC";

        case SAFETY_LOCK:
            return "SAFETY_LOCK";

        default:
            return "UNKNOWN";
    }
}

void DebugManager::printSystemStatus()
{

    printHeader("SYSTEM STATUS");

    Serial.print("Mode            : ");
    Serial.println(
        getModeName(
            systemState.currentMode));

    printBool(
        "Manual Mode",
        systemState.manualMode);

    printBool(
    "WiFi Connected",
    systemState.wifiConnected);

    printBool(
    "Firebase Connected",
    systemState.firebaseConnected);

    printBool(
        "Reservoir Lock",
        systemState.reservoirLocked);

    printBool(
        "Fog Cycle",
        actuatorManager.isOn(FOGGER));

    Serial.print("pH Direction    : ");

    switch(systemState.phDirection)
    {
        case PH_NONE:
            Serial.println("NONE");
            break;

        case PH_UP:
            Serial.println("UP");
            break;

        case PH_DOWN:
            Serial.println("DOWN");
            break;
    }

    Serial.print("EC Dose Time    : ");
    Serial.print(systemState.ecDoseTime / 1000);
    Serial.println(" sec");

    Serial.print("PH Attempts     : ");
    Serial.println(systemState.phAttempts);

    Serial.print("EC Attempts     : ");
    Serial.println(systemState.ecAttempts);

    printBool(
    "Safety Lock",
    systemState.currentMode ==
    SAFETY_LOCK);


    Serial.print("Min PH          : ");
    Serial.println(systemState.minPH);

    Serial.print("Max PH          : ");
    Serial.println(systemState.maxPH);

    Serial.print("Min EC          : ");
    Serial.println(systemState.minEC);

    Serial.print("Light ON        : ");
    Serial.print(systemState.lightOnHour);
    Serial.print(":");
    Serial.println(systemState.lightOnMinute);

    Serial.print("Light OFF       : ");
    Serial.print(systemState.lightOffHour);
    Serial.print(":");
    Serial.println(systemState.lightOffMinute);

    Serial.print("RTC Time        : ");

    Serial.print(
        rtcManager.getHour());

    Serial.print(":");

    Serial.print(
        rtcManager.getMinute());

    Serial.print(":");

    Serial.println(
        rtcManager.getSecond());

        printSeparator();
    }

void DebugManager::printAlerts()
{
    printHeader("ALERT STATUS");

    printBool(
        "Low Water",
        alertState.lowWater);

    printBool(
        "EC Low",
        alertState.ecLow);

    printBool(
        "pH Out Of Range",
        alertState.phOutOfRange);

    printBool(
        "Water Temp OOR",
        alertState.waterTempOutOfRange);

    printBool(
        "High Air Temp",
        alertState.highTemperature);

    printBool(
        "Sensor Fault",
        alertState.sensorFault);

    printSeparator();
}

void DebugManager::printRTC()
{
    printHeader("RTC STATUS");

    Serial.print("Current Time : ");

    if(rtcManager.getHour() < 10)
        Serial.print("0");

    Serial.print(rtcManager.getHour());

    Serial.print(":");

    if(rtcManager.getMinute() < 10)
        Serial.print("0");

    Serial.print(rtcManager.getMinute());

    Serial.print(":");

    if(rtcManager.getSecond() < 10)
        Serial.print("0");

    Serial.println(rtcManager.getSecond());

    Serial.print("Light ON     : ");

    Serial.print(systemState.lightOnHour);

    Serial.print(":");

    Serial.println(systemState.lightOnMinute);

    Serial.print("Light OFF    : ");

    Serial.print(systemState.lightOffHour);

    Serial.print(":");

    Serial.println(systemState.lightOffMinute);

    printSeparator();
}