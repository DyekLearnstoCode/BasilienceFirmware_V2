#include "Config.h"
#include "Types.h"
#include "Globals.h"

#include "SensorManager.h"
#include "ActuatorManager.h"
#include "AutomationManager.h"
#include "FirebaseManager.h"
#include "AlertManager.h"
#include "StartupManager.h"
#include "MixingManager.h"
#include "SafetyManager.h"

SensorManager sensorManager;
ActuatorManager actuatorManager;
AutomationManager automationManager;
FirebaseManager firebaseManager;
AlertManager alertManager;
StartupManager startupManager;
MixingManager mixingManager;
SafetyManager safetyManager;

void setup()
{
    Serial.begin(115200);

    actuatorManager.begin();
    sensorManager.begin();
    firebaseManager.begin();
    startupManager.begin();
    mixingManager.begin();
    alertManager.begin();
    automationManager.begin();
    safetyManager.begin();
}

void loop()
{
    sensorManager.update();

    Serial.print("Temperature: ");
    Serial.print(sensors.temperature);

    Serial.print(" °C | Humidity: ");
    Serial.print(sensors.humidity);

    Serial.print(" % | Water Temp: ");
    Serial.print(sensors.waterTemp);

    Serial.print(" °C | Water Level: ");
    Serial.print(sensors.waterLevel);

    Serial.println(" %");

    automationManager.update();
    alertManager.update();
    firebaseManager.update();

    delay(1000);
}