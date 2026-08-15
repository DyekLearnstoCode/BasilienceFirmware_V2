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

    // Local plant protection is initialized before any network or TLS path.
    // Defaults/persisted settings, safe GPIO states, sensors, RTC, safety, and
    // automation are therefore available even when Firebase is unreachable.
    firebaseManager.loadPersistedSettings();
    actuatorManager.begin();
    sensorManager.begin();
    rtcManager.begin();
    alertManager.begin();
    safetyManager.begin();
    automationManager.begin();
    debugManager.begin();

    Serial.println("[CONTROL] Local automation and safety ready");

    wifiManager.begin();
    Serial.print("ESP32 MAC: ");
    Serial.println(WiFi.macAddress());
}

void loop()
{
    // Provisioning is a local, latency-sensitive mode. Service DNS/HTTP first and
    // skip every Firebase/SSL path while the setup AP owns the radio. Local
    // sensor, safety, automation, and actuator enforcement must still run.
    wifiManager.update();
    const bool provisioningMode = wifiManager.isProvisioningMode();

    sensorManager.update();
    rtcManager.update();

    if (!systemState.sensorTestEnabled)
    {
        automationManager.update();
        safetyManager.update();
    }
    actuatorManager.update();

    // All optional Firebase polling runs after local sensing, control, safety,
    // and actuator enforcement. Newly received mock/test commands take effect
    // on the next local control iteration.

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
    if (!firebaseInitialized && wifiManager.isConnected())
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
