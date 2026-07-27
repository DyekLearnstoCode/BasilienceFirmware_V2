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
    rtcManager.begin();

    firebaseManager.begin();

    automationManager.begin();

    debugManager.begin();
}

void loop()
{
    sensorManager.update();
    rtcManager.update();

    automationManager.update();

    firebaseManager.update();

    debugManager.update();
}
