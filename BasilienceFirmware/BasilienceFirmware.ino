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
