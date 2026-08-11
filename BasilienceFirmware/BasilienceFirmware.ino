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

bool firebaseInitialized = false;

void setup()
{

    Serial.begin(115200);

    // Restore the last validated automation configuration before networking so
    // an offline boot does not silently forget the user's accepted settings.
    firebaseManager.loadPersistedSettings();
    wifiManager.begin();
    Serial.print("ESP32 MAC: ");
Serial.println(WiFi.macAddress());
    if (!wifiManager.isProvisioningMode())
    {
        firebaseManager.begin();
        firebaseInitialized = true;
    }
    actuatorManager.begin();

    sensorManager.begin();
    rtcManager.begin();


    automationManager.begin();

    debugManager.begin();
    safetyManager.begin();
}

void loop()
{
    // Provisioning is a local, latency-sensitive mode. Service DNS/HTTP first and
    // skip every Firebase/SSL path while the setup AP owns the radio. Local
    // sensor, safety, automation, and actuator enforcement must still run.
    wifiManager.update();
    const bool provisioningMode = wifiManager.isProvisioningMode();

    // Read a pending mock command before choosing the effective sensor source.
    // This guarantees automation consumes the same dataset acknowledged in status/mockData.
    if (!provisioningMode)
    {
        firebaseManager.syncMockSensors();
    }

    sensorManager.update();
    rtcManager.update();

    automationManager.update();
    safetyManager.update();
    actuatorManager.update();

    if (provisioningMode)
    {
        // Keep the AP HTTP/DNS service responsive without suspending local
        // safety checks or actuator watchdogs.
        wifiManager.update();
        debugManager.update();
        return;
    }

    // A saved network can recover while the setup AP is active. Firebase was
    // intentionally never started on that boot, so initialize it only now.
    if (!firebaseInitialized)
    {
        firebaseManager.begin();
        firebaseInitialized = true;
    }

    firebaseManager.update();
    if (wifiManager.isProvisioningMode())
    {
        wifiManager.update();
        return;
    }

    debugManager.update();
}
