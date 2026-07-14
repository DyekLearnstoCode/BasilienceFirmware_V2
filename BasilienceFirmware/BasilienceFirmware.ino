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
#include "DebugManager.h"

SensorManager sensorManager;
ActuatorManager actuatorManager;
AutomationManager automationManager;
FirebaseManager firebaseManager;
AlertManager alertManager;
StartupManager startupManager;
MixingManager mixingManager;
SafetyManager safetyManager;
DebugManager debugManager;

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
    debugManager.begin();
}

void loop()
{
    sensorManager.update();

    automationManager.update();

    alertManager.update();

    firebaseManager.update();

    debugManager.update();
}
