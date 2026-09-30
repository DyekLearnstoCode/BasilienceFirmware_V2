
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
#include <esp_system.h>

bool firebaseInitialized = false;

// Boot diagnostic only. The ROM banner cannot tell a brownout from a plain
// software reset (both print "rst:0x3 (SW_RESET)"), but esp_reset_reason()
// can, so this names the cause in plain text on every boot.
static const char* resetReasonName(esp_reset_reason_t reason)
{
    switch (reason)
    {
        case ESP_RST_POWERON:   return "ESP_RST_POWERON";
        case ESP_RST_EXT:       return "ESP_RST_EXT";
        case ESP_RST_SW:        return "ESP_RST_SW";
        case ESP_RST_PANIC:     return "ESP_RST_PANIC";
        case ESP_RST_INT_WDT:   return "ESP_RST_INT_WDT";
        case ESP_RST_TASK_WDT:  return "ESP_RST_TASK_WDT";
        case ESP_RST_WDT:       return "ESP_RST_WDT";
        case ESP_RST_DEEPSLEEP: return "ESP_RST_DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "ESP_RST_BROWNOUT";
        case ESP_RST_SDIO:      return "ESP_RST_SDIO";
        case ESP_RST_UNKNOWN:   return "ESP_RST_UNKNOWN";
        default:                return "OTHER";
    }
}

void setup()
{

    Serial.begin(115200);

    const esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.print("[BOOT] Reset reason raw=");
    Serial.print((int)resetReason);
    Serial.print(" name=");
    Serial.println(resetReasonName(resetReason));

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

    // Serial Diagnostics / Observability pass (section 14): one compact
    // configuration summary, now that every local manager's own begin() has
    // run and RTC/harvest-cache/settings state reflects this boot - still
    // before Wi-Fi/Firebase start, so it never depends on connectivity.
    debugManager.printBootSummary();
    Serial.println("[BOOT] Local automation initialized");
    Serial.println("[BOOT] Waiting for WiFi...");

    wifiManager.begin();

    // WiFi.macAddress() reads back all-zero here whenever this boot has no
    // saved credentials (goes straight into WiFi.mode(WIFI_AP)-only
    // provisioning, so the STA netif is never started) - see the task report
    // for the full trace. getFormattedMacAddress()/getMacAddress() read the
    // hardware MAC directly via esp_read_mac(), which works regardless of
    // WiFi mode/state. A "" result means readHardwareStaMac() already logged
    // "[IDENTITY] ERROR: ..." - nothing further to print here.
    String staMac = firebaseManager.getFormattedMacAddress();
    if (!staMac.isEmpty())
    {
        Serial.print("[IDENTITY] STA MAC: ");
        Serial.println(staMac);
        Serial.print("[IDENTITY] Provisioning key: ");
        Serial.println(firebaseManager.getMacAddress());
    }

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

    // Must come after actuatorManager.begin() above: reboot-recovery checks
    // whether the last confirmed state was ON, and by this point FOGGER has
    // already been physically/locally reset to OFF, so any recovery
    // closeout event this records is accurate.
    foggingEventQueue.begin();
}

void loop()
{
    // Serial Diagnostics / Observability pass (sections 6/7): recorded first,
    // before any other work this iteration, so the measured gap is how long
    // the whole previous iteration actually took, including everything below.
    debugManager.recordLoopTick();

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

    // All optional Firebase polling runs after local sensing, control, safety,
    // and actuator enforcement. Newly received mock/test commands take effect
    // on the next local control iteration.

    if (provisioningMode)
    {
        // Keep the AP HTTP/DNS service responsive without suspending local
        // safety checks or actuator watchdogs.
        wifiManager.update();
        firebaseManager.noteProvisioningActive();
        debugManager.update();
        return;
    }

    // A saved network can recover while the setup AP is active. Firebase was
    // intentionally never started on that boot, so it is armed only now.
    // begin() does NO network I/O and returns immediately: the first cloud
    // connection (network preflight -> authentication -> database
    // initialization) is advanced a bounded step per loop() iteration by
    // update() below, so an unreachable or slow cloud can never hold up the
    // sensing/safety/automation/actuator work above. Cloud readiness is
    // reported by systemState.firebaseConnected, which stays false until every
    // startup step has finished - Wi-Fi being connected does not imply it.
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
