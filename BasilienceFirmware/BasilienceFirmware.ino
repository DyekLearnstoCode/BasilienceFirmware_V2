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
#include "GsmRawUartTest.h"

bool firebaseInitialized = false;

// TEMPORARY hardware bring-up diagnostic - see Config.h GSM_RAW_UART_TEST.
// Only ever begin()/update()'d when that flag is true; otherwise never
// touched, so it cannot affect production behavior or contend for the GSM
// UART with GsmManager.
GsmRawUartTest gsmRawUartTest;

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

    // GSM_RAW_UART_TEST is a constexpr bool, not a preprocessor macro - the
    // C preprocessor cannot see it, so #if here would silently treat it as
    // undefined (== 0) regardless of its actual value. A plain runtime/
    // constexpr-foldable if/else is what SECURE_DEVICE_AUTH_REQUIRED (the
    // same kind of Config.h flag) already correctly uses elsewhere in this
    // codebase; the compiler still eliminates the untaken branch entirely
    // since the condition is constexpr, so this costs nothing extra.
    if (GSM_RAW_UART_TEST)
    {
        // Raw hardware bring-up diagnostic: the production GSM/notification
        // pipeline below is intentionally NOT initialized while this is active,
        // so nothing else ever touches the GSM UART/pins concurrently.
        Serial.println("[GSM-TEST] RAW UART DIAGNOSTIC MODE ACTIVE");
        gsmRawUartTest.begin();
    }
    else
    {
        Serial.println("[GSM] Production GSM manager active");

        // GSM is a connectivity concern like Wi-Fi/Firebase, not core plant
        // safety, so it is initialized alongside them. begin() is non-blocking -
        // it only opens the UART and fires the first probe.
        gsmManager.begin();

        // Recipient/harvest caches and the notification queue must be available
        // before the first alert transition can be observed, so they are
        // initialized last in this connectivity group, still non-blocking (pure
        // NVS reads).
        smsRecipientCache.begin();
        harvestScheduleCache.begin();
        notificationManager.begin();
    }
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

    // See setup() for why this is a runtime if/else rather than #if.
    if (GSM_RAW_UART_TEST)
    {
        // Raw UART diagnostic only - see setup(). Non-blocking (millis()-driven),
        // so this never delays sensing/control above or below, same as the
        // production path it replaces.
        gsmRawUartTest.update();
    }
    else
    {
        // GSM registration is independent of Wi-Fi: it is serviced every
        // iteration regardless of provisioning/connectivity state and is never
        // power-cycled on a Wi-Fi transition. Its update() is bounded/non-
        // blocking, so this never delays sensing/control below.
        gsmManager.update();

        // Notification queue/SMS fan-out is likewise independent of Wi-Fi and
        // provisioning state - it must keep running (e.g. to finish delivering
        // an in-flight SMS) even while the setup AP owns the radio. Non-
        // blocking, so this never delays sensing/control above or below.
        notificationManager.update();
    }

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
