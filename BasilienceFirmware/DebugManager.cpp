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

    // Serial Monitor Focus Mode: the periodic round-robin dashboards below
    // are exactly the kind of high-volume generic output an isolated
    // controller test does not want competing with its own focused event
    // logs (see shouldPrintDebug()'s own comment - SYSTEM is never true
    // while a controller is isolated). Suppressed entirely rather than
    // replaced with a mode-specific summary - the focused event logs added
    // at each controller's own log sites already serve that role. NONE
    // (normal/full-system operation) is completely unaffected.
    if (!shouldPrintDebug(DebugCategory::SYSTEM))
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

    printFloat(
        "Water Level Depth",
        sensors.waterLevelCm,
        "cm",
        2);

    printFloat(
        "Water Volume",
        sensors.waterVolumeLiters,
        "L",
        2);

    printFloat(
        "Water Level Distance",
        sensors.waterLevelDistanceCm,
        "cm",
        2);

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
        "Canopy Fan",
        actuatorManager.isOn(CANOPY_FAN));

    printBool(
        "Peltier",
        actuatorManager.isOn(PELTIER));

    // Speed (PWM duty, 0-100) only actually varies for the two PWM-capable
    // actuators - see ActuatorManager::isPwmActuator(). Printed here rather
    // than folded into the ON/OFF lines above so a commanded-but-unapplied
    // speed change is visible on its own.
    printInteger(
        "Canopy Fan Speed",
        actuatorManager.getStatus(CANOPY_FAN).speed,
        "%");

    printInteger(
        "Blower Speed",
        actuatorManager.getStatus(BLOWER).speed,
        "%");

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

// ======================================================
// Serial Monitor Focus Mode
// ======================================================
// See DebugManager.h's own comments. All three methods read
// systemState.automationTestSubsystem fresh on every call - no cached/
// compile-time state - so they track a live mode change immediately.

bool DebugManager::shouldPrintDebug(DebugCategory category) const
{
    if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
        return true;

    switch (systemState.automationTestSubsystem)
    {
        case AutomationTestSubsystem::STARTUP:
            // Startup's own phase/timer diagnostics, plus water depth - the
            // pre-startup refill decision and accepted waterLevelCm are
            // explicitly in scope even though REFILL is not the isolated
            // controller. DHT deliberately excluded: startup fogging does
            // not consume it (SafetyManager::canFog()).
            return category == DebugCategory::STARTUP ||
                   category == DebugCategory::WATER;

        case AutomationTestSubsystem::REFILL:
            return category == DebugCategory::WATER;

        case AutomationTestSubsystem::PH:
            return category == DebugCategory::PH;

        case AutomationTestSubsystem::EC:
            return category == DebugCategory::EC;

        case AutomationTestSubsystem::COOLING:
            return category == DebugCategory::COOLING;

        case AutomationTestSubsystem::FOGGING:
            // DHT is optional for fogging (cadence selection only) but its
            // availability/stale status is explicitly requested - the raw
            // read diagnostics are already throttled to one line per 5s
            // (DHT_RAW_DIAGNOSTIC_INTERVAL_MS) so this is not the per-pH/EC-
            // sample spam the task explicitly asks to avoid re-testing here.
            return category == DebugCategory::FOGGING ||
                   category == DebugCategory::DHT;

        case AutomationTestSubsystem::CANOPY:
            return category == DebugCategory::CANOPY ||
                   category == DebugCategory::DHT;

        case AutomationTestSubsystem::GROW_LIGHT:
            return category == DebugCategory::LIGHT;

        default:
            return false;
    }
}

bool DebugManager::shouldPrintActuator(Actuator actuator) const
{
    if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
        return true;

    switch (systemState.automationTestSubsystem)
    {
        case AutomationTestSubsystem::STARTUP:
            return actuator == FOGGER || actuator == BLOWER || actuator == SOLENOID;

        case AutomationTestSubsystem::REFILL:
            return actuator == SOLENOID;

        case AutomationTestSubsystem::PH:
            return actuator == PH_UP_PUMP || actuator == PH_DOWN_PUMP ||
                   actuator == CIRCULATION_PUMP;

        case AutomationTestSubsystem::EC:
            // SOLENOID included - EC dilution actuates it (see
            // AutomationManager::handleDosingEC()'s EC_DILUTE branch).
            return actuator == GROW_PUMP || actuator == BLOOM_PUMP ||
                   actuator == CIRCULATION_PUMP || actuator == SOLENOID;

        case AutomationTestSubsystem::COOLING:
            return actuator == PELTIER || actuator == CIRCULATION_PUMP;

        case AutomationTestSubsystem::FOGGING:
            return actuator == FOGGER || actuator == BLOWER;

        case AutomationTestSubsystem::CANOPY:
            return actuator == CANOPY_FAN;

        case AutomationTestSubsystem::GROW_LIGHT:
            return actuator == GROW_LIGHT;

        default:
            return false;
    }
}

bool DebugManager::shouldPrintStateTransition(SystemMode fromMode, SystemMode toMode) const
{
    if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
        return true;

    // Always-critical / always-common, regardless of which controller (if
    // any) is isolated: a safety lock is a system-wide event by definition,
    // and NORMAL/SENSOR_STABILIZATION are the shared resting/boot states
    // every controller transitions through.
    if (fromMode == SAFETY_LOCK || toMode == SAFETY_LOCK ||
        fromMode == SENSOR_STABILIZATION || toMode == SENSOR_STABILIZATION ||
        toMode == NORMAL)
        return true;

    switch (systemState.automationTestSubsystem)
    {
        case AutomationTestSubsystem::STARTUP:
            return fromMode == STARTUP || toMode == STARTUP;

        case AutomationTestSubsystem::REFILL:
            return fromMode == REFILLING || toMode == REFILLING;

        case AutomationTestSubsystem::PH:
            return fromMode == DOSING_PH || toMode == DOSING_PH ||
                   fromMode == STABILIZING_PH || toMode == STABILIZING_PH;

        case AutomationTestSubsystem::EC:
            return fromMode == DOSING_EC || toMode == DOSING_EC ||
                   fromMode == STABILIZING_EC || toMode == STABILIZING_EC;

        default:
            // COOLING/FOGGING/CANOPY/GROW_LIGHT have no dedicated SystemMode
            // of their own - they run continuously inside NORMAL (already
            // covered by the toMode == NORMAL rule above), so there is no
            // additional state transition to show for them.
            return false;
    }
}

