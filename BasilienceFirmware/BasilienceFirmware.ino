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
    
    wifiManager.begin();
    Serial.print("ESP32 MAC: ");
Serial.println(WiFi.macAddress());
    firebaseManager.begin();
    actuatorManager.begin();

    sensorManager.begin();
    rtcManager.begin();


    automationManager.begin();

    debugManager.begin();
    safetyManager.begin();
}

void loop()
{
    sensorManager.update();
    rtcManager.update();

    automationManager.update();
    actuatorManager.update();

    firebaseManager.update();

    debugManager.update();
    wifiManager.update();
}
