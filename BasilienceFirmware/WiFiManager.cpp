#include "WiFiManager.h"

#include "Globals.h"

// Forward declaration: defined below, registered from begin() before any
// connection attempt so every STA event - including the very first - is
// captured with its disconnect reason code.
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);

namespace
{
    // Set immediately before every WiFi.disconnect() this class issues
    // itself (see WiFiManager::disconnectRadio), read and cleared once by
    // onWiFiEvent's DISCONNECTED case. If a disconnect event arrives with
    // this NOT set, the drop was not requested by this firmware (AP-side
    // deauth, RF loss, or - before this fix - a third party such as the
    // Firebase client library calling WiFi.reconnect() on its own).
    bool g_selfInitiatedDisconnectPending = false;

    // Best-effort decode of the ESP-IDF wifi_err_reason_t values actually
    // defined in this SDK (esp_wifi_types_generic.h) - not exhaustive, but
    // covers every reason this diagnosis is likely to actually see. Falls
    // back to "unknown" rather than guessing.
    const char* wifiDisconnectReasonName(uint8_t reason)
    {
        switch (reason)
        {
            case 1:   return "UNSPECIFIED";
            case 2:   return "AUTH_EXPIRE";
            case 3:   return "AUTH_LEAVE";
            case 4:   return "ASSOC_EXPIRE/DISASSOC_INACTIVITY";
            case 5:   return "ASSOC_TOOMANY";
            case 6:   return "NOT_AUTHED";
            case 7:   return "NOT_ASSOCED";
            case 8:   return "ASSOC_LEAVE";               // station-initiated leave (e.g. esp_wifi_disconnect())
            case 9:   return "ASSOC_NOT_AUTHED";
            case 10:  return "DISASSOC_PWRCAP_BAD";
            case 11:  return "DISASSOC_SUPCHAN_BAD";
            case 13:  return "IE_INVALID";
            case 14:  return "MIC_FAILURE";
            case 15:  return "4WAY_HANDSHAKE_TIMEOUT";
            case 16:  return "GROUP_KEY_UPDATE_TIMEOUT";
            case 39:  return "TIMEOUT";
            case 46:  return "PEER_INITIATED";
            case 47:  return "AP_INITIATED";
            case 200: return "BEACON_TIMEOUT";
            case 201: return "NO_AP_FOUND";
            case 202: return "AUTH_FAIL";
            case 203: return "ASSOC_FAIL";
            case 204: return "HANDSHAKE_TIMEOUT";
            case 205: return "CONNECTION_FAIL";
            case 206: return "AP_TSF_RESET";
            case 207: return "ROAMING";
            default:  return "unknown";
        }
    }
}

void WiFiManager::begin()
{
    WiFi.onEvent(onWiFiEvent);

    // ROOT CAUSE of "[WIFI] reconnect handled by ESP32 auto-reconnect" still
    // appearing on real hardware after the previous fix: that earlier fix
    // (Firebase.reconnectNetwork(false)) only stopped the Firebase client
    // library's own reconnect calls. It never touched the ESP32 Arduino
    // core's OWN native auto-reconnect, which is a SEPARATE mechanism -
    // STAClass's constructor defaults _autoReconnect to true
    // (libraries/WiFi/src/STA.cpp:231, ESP32 core 3.1.3), and nothing in
    // this firmware ever called setAutoReconnect(false) to turn it off.
    // On a disconnect with a "reconnectable" reason, STA.cpp's internal
    // _onStaArduinoEvent (~line 156-164) synchronously calls disconnect()
    // then connect() itself, entirely outside WiFiManager's state machine -
    // the exact second reconnect owner this architecture is supposed to
    // rule out. That same synchronous re-entrant disconnect()/connect()
    // pair is also the most likely reason the [WIFI-EVENT] logs (connected,
    // got-IP, AND disconnected - all three, not just disconnected) never
    // printed cleanly: the event object/dispatch gets reused before this
    // firmware's own onWiFiEvent() has a clean chance to run. This call is
    // set once, here, at boot - the flag lives on the WiFi singleton
    // constructed at static-init time, so WiFi.mode()/WiFi.begin() calls
    // later never reset it back to its default-true state.
    WiFi.setAutoReconnect(false);

    // Sibling fix to setAutoReconnect(false) above, same root cause: the
    // ESP-IDF WiFi driver keeps its OWN internal, NVS-backed STA config
    // (ssid/password), entirely separate from this app's own Preferences
    // "wifi" namespace below. WiFi.persistent() defaults to true, so every
    // WiFi.begin(ssid, password) call this firmware makes - including every
    // FAILED attempt against an old/replaced network from the self-heal
    // retry loop or a normal reconnect - also silently overwrites that
    // internal store. An explicit WiFi.begin(ssid, password) call always
    // wins over it, but leaving persistence on means stale credentials sit
    // in a second, undocumented location this app's own saveCredentials()/
    // loadCredentials() never touches or clears - exactly the kind of
    // hidden state the setAutoReconnect(false) fix above was written to
    // rule out for auto-reconnect. Disabling it here makes this app's own
    // Preferences "wifi" namespace the SOLE source of truth for every
    // connection attempt, with no other cached fallback able to take effect.
    WiFi.persistent(false);

    preferences.begin("wifi", false);
    firebaseResumePending = preferences.getBool("resumeFirebase", false);
    if (firebaseResumePending)
    {
        preferences.remove("resumeFirebase");
    }
    preferences.end();

    if (!loadCredentials())
    {
        Serial.println("No saved WiFi credentials.");
        enterAutomaticProvisioningMode();
        initialConnectionAttempt = false;
        return;
    }

    // Kick off the connection attempt without blocking setup(). loop() starts
    // immediately so local sensing/safety/automation run from the first
    // iteration; update() drives the STA connection to completion in the
    // background through the same state machine a mid-run drop uses.
    startConnectionAttempt();

    initialConnectionAttempt = false;
}

// The ONLY saved-credential STA initiation path in normal operation. Every
// association attempt - first boot, retry after timeout, retry after a drop -
// goes through here exactly once per attempt, which is what guarantees the
// radio is never reconfigured while a previous attempt is still in flight.
void WiFiManager::startConnectionAttempt()
{
    if (!hasCredentials())
    {
        wifiState = WifiState::IDLE;
        return;
    }

    const char* previousState = stateName();

    Serial.print("[WIFI] Connecting to ");
    Serial.println(ssid);
    Serial.println("[WIFI] reconnect initiated by firmware");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());

    connectionStartedAt = millis();
    lastReconnectAttempt = connectionStartedAt;
    wifiState = WifiState::CONNECTING;
    logStateChange(previousState, "CONNECTING", "startConnectionAttempt");

    // The cumulative outage window starts with the FIRST attempt of an
    // outage, not after it fails - "how long has this device been off the
    // network" is the question the fallback timeout is meant to answer. The
    // zero-check is what makes it cumulative: every later retry passes
    // through here and must leave the original start time alone.
    if (recoveryStartedAt == 0)
    {
        recoveryStartedAt = connectionStartedAt;
    }

    Serial.println("[WIFI] Connection attempt in progress...");
}

void WiFiManager::enterConnectedState()
{
    const char* previousState = stateName();

    // RETRY_WAIT only ever exits into CONNECTING via startConnectionAttempt()
    // (see its state machine in update()) - so reaching CONNECTED directly
    // from RETRY_WAIT, without ever passing through CONNECTING, means
    // something other than this firmware's own retry logic re-established
    // the link (ESP32/library-level auto-reconnect). Diagnostic only.
    if (wifiState == WifiState::RETRY_WAIT)
    {
        Serial.println("[WIFI] reconnect handled by ESP32 auto-reconnect");
    }

    wifiState = WifiState::CONNECTED;
    connectionStartedAt = 0;
    lastReconnectAttempt = 0;

    // A successful association ends the cumulative outage window; the next
    // outage starts measuring from scratch.
    recoveryStartedAt = 0;
    systemState.wifiConnected = true;

    if (recoveryInProgress)
    {
        Serial.println("[WIFI] Saved network restored");
        recoveryInProgress = false;
    }

    Serial.println("[WIFI] Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    logStateChange(previousState, "CONNECTED", "enterConnectedState");
}


// LEGACY - not part of the connection state machine and unreachable in this
// build: its only caller chain is reconnect() <- updateCredentialsSafely(),
// which has no callers. Left in place deliberately (no dead-code cleanup in
// this task). It must not be wired back up without being rewritten on top of
// startConnectionAttempt(); it blocks for up to RECOVERY_TIMEOUT and would
// bypass every guarantee below.
bool WiFiManager::connect()
{
    if (!hasCredentials())
    {
        Serial.println("No WiFi credentials available.");

        systemState.wifiConnected = false;
        enterAutomaticProvisioningMode();
        return false;
    }

    // Already connected
    if (WiFi.status() == WL_CONNECTED)
    {
        systemState.wifiConnected = true;

        return true;
    }

    Serial.println();
    Serial.print("[WIFI] Connecting to ");
    Serial.print(ssid);
    Serial.println("...");

    WiFi.mode(WIFI_STA);

    WiFi.begin(
        ssid.c_str(),
        password.c_str());

    unsigned long startTime = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - startTime >= RECOVERY_TIMEOUT)
        {
            Serial.println();
            Serial.println("[WIFI] Connection failed");

            systemState.wifiConnected = false;
            enterAutomaticProvisioningMode();

            return false;
        }

        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("[WIFI] Connected");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    systemState.wifiConnected = true;

    return true;
}

void WiFiManager::disconnect()
{
    disconnectRadio(false, false);

    systemState.wifiConnected = false;

    Serial.println("WiFi Disconnected");
}

void WiFiManager::disconnectRadio(bool wifioff, bool eraseap)
{
    g_selfInitiatedDisconnectPending = true;
    WiFi.disconnect(wifioff, eraseap);
}

const char* WiFiManager::stateName() const
{
    switch (wifiState)
    {
        case WifiState::IDLE:       return "IDLE";
        case WifiState::CONNECTING: return "CONNECTING";
        case WifiState::CONNECTED:  return "CONNECTED";
        case WifiState::RETRY_WAIT: return "RETRY_WAIT";
    }
    return "UNKNOWN";
}

void WiFiManager::logStateChange(const char* from, const char* to, const char* reason) const
{
    Serial.print("[WIFI] state ");
    Serial.print(from);
    Serial.print(" -> ");
    Serial.print(to);
    Serial.print(" reason=");
    Serial.println(reason);
}

// Transition/event logging only - never spammed per-loop. GOT_IP/STA
// CONNECTED are single events per association; DISCONNECTED carries full
// diagnostic context since that is the one this whole task exists to
// explain.
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    switch (event)
    {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.print("[WIFI-EVENT] STA connected t=");
            Serial.println(millis());
            break;

        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("[WIFI-EVENT] Got IP: ");
            Serial.print(WiFi.localIP());
            Serial.print(" t=");
            Serial.println(millis());
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        {
            const uint8_t reason = info.wifi_sta_disconnected.reason;
            const bool selfInitiated = g_selfInitiatedDisconnectPending;
            g_selfInitiatedDisconnectPending = false;

            Serial.print("[WIFI-EVENT] Disconnected reason=");
            Serial.print(reason);
            Serial.print(" (");
            Serial.print(wifiDisconnectReasonName(reason));
            Serial.println(")");

            Serial.print("[WIFI-EVENT]   origin=");
            Serial.println(selfInitiated ? "firmware-initiated" : "external");

            Serial.print("[WIFI-EVENT]   previous state=");
            Serial.println(wifiManager.stateName());

            // RSSI is only meaningful while associated; ESP32 Arduino
            // returns 0 once the STA has already dropped, which this
            // reports plainly rather than presenting as a real reading.
            const int rssi = WiFi.RSSI();
            Serial.print("[WIFI-EVENT]   rssi=");
            if (rssi == 0) Serial.println("n/a");
            else Serial.println(rssi);

            // Best-effort context, not a live "is a request in flight right
            // now" flag (FirebaseManager has no such flag to read, and
            // adding one across every RTDB call site is out of scope for a
            // Wi-Fi diagnosis) - reflects the last known Firebase health
            // state at the moment of this disconnect.
            Serial.print("[WIFI-EVENT]   firebaseConnected=");
            Serial.println(systemState.firebaseConnected ? "true" : "false");

            Serial.print("[WIFI-EVENT]   t=");
            Serial.println(millis());
            break;
        }

        default:
            break;
    }
}

// LEGACY - see connect() above. Unreachable in this build.
bool WiFiManager::reconnect()
{
    disconnect();

    delay(500);

    return connect();
}

bool WiFiManager::saveCredentials(
    const String& ssid,
    const String& password)
{
    preferences.begin("wifi", false);

    size_t ssidBytes = preferences.putString("ssid", ssid);
    size_t passwordBytes = preferences.putString("password", password);
    // Diagnostic for the failure mode this function fixes: putString()
    // internally does nvs_set_str() THEN nvs_commit(), returning 0 if
    // either fails (e.g. a full/fragmented NVS partition) - only logged via
    // ESP-IDF's own log_e(), invisible at this project's serial log level.
    // freeEntries() surfaces that same health signal on this app's own log
    // instead, so a save failure is diagnosable without raising the core
    // debug level.
    size_t freeEntries = preferences.freeEntries();

    preferences.end();

    Serial.print("[WIFI] saveCredentials: ssidBytes=");
    Serial.print(ssidBytes);
    Serial.print(" passwordBytes=");
    Serial.print(passwordBytes);
    Serial.print(" nvsFreeEntries=");
    Serial.println(freeEntries);

    // putString() returning 0 was previously never checked here, so a
    // write that silently failed (NVS full/corrupted, or nvs_commit()
    // failing after a successful nvs_set_str()) still reported success to
    // this function's caller and to the /setup HTTP handler, leaving
    // whatever credentials already existed in NVS untouched - the confirmed
    // cause of "credentials always revert to the previous network."
    // /setup already rejects an empty SSID before ever calling this, so
    // ssidBytes==0 here is unambiguously a real failure, not a legitimate
    // empty value; password may legitimately be empty (open networks), so
    // passwordBytes==0 only counts as a failure when a non-empty password
    // was actually submitted.
    if (ssidBytes == 0 || (password.length() > 0 && passwordBytes == 0))
    {
        Serial.println("[WIFI] saveCredentials FAILED: NVS write did not persist - keeping previous in-memory credentials");
        return false;
    }

    this->ssid = ssid;
    this->password = password;

    return true;
}

bool WiFiManager::updateCredentialsSafely(
    const String& newSsid,
    const String& newPassword)
{
    String oldSsid = this->ssid;
    String oldPassword = this->password;

    Serial.println("Testing new Wi-Fi credentials...");
    saveCredentials(newSsid, newPassword);

    if (reconnect())
    {
        Serial.println("Successfully connected to new network.");
        return true;
    }
    else
    {
        Serial.println("Failed to connect to new network. Restoring previous credentials.");
        saveCredentials(oldSsid, oldPassword);
        reconnect();
        return false;
    }
}

void WiFiManager::clearCredentials()
{
    preferences.begin("wifi", false);

    preferences.remove("ssid");
    preferences.remove("password");

    preferences.end();

    ssid.clear();
    password.clear();

    Serial.println("WiFi credentials cleared.");
}

bool WiFiManager::hasCredentials() const
{
    // An empty password is valid for an open target Wi-Fi network.
    return !ssid.isEmpty();
}

bool WiFiManager::isConnected() const
{
    return WiFi.status() == WL_CONNECTED;
}

bool WiFiManager::isProvisioningMode() const
{
    return provisioningMode != ProvisioningMode::NONE;
}

WiFiManager::ProvisioningMode WiFiManager::getProvisioningMode() const
{
    return provisioningMode;
}

bool WiFiManager::consumeFirebaseResumePending()
{
    bool pending = firebaseResumePending;
    firebaseResumePending = false;
    return pending;
}

String WiFiManager::getSSID() const
{
    return ssid;
}

bool WiFiManager::loadCredentials()
{
    preferences.begin("wifi", true);

    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");

    preferences.end();

    if (hasCredentials())
    {
        Serial.println("WiFi credentials loaded.");
        Serial.print("SSID: ");
        Serial.println(ssid);

        return true;
    }

    return false;
}

void WiFiManager::update()
{
    if (isProvisioningMode())
    {
        dnsServer.processNextRequest();
        server.handleClient();

        // MANUAL provisioning is a user actively submitting NEW credentials
        // through this same setup AP (WifiConfigFragment's POST /setup) - the
        // self-heal retry below has no business running here, and actively
        // breaks that submission. ESP32's AP and STA share one radio: every
        // AP_RECONNECT_INTERVAL (30s) this used to force WIFI_AP_STA and call
        // WiFi.begin() to retry the OLD saved network, an association attempt
        // that stays in flight for up to RECOVERY_TIMEOUT (20s) - i.e. the
        // softAP's radio was destabilized roughly 2/3 of the time. If the
        // phone's POST /setup landed during that window, the request would
        // time out or the connection would reset (confirmed bug: "credentials
        // submit fails"). Only FALLBACK provisioning (unattended, entered
        // because the device itself lost its known network) should keep
        // trying to self-heal in the background.
        if (provisioningMode == ProvisioningMode::MANUAL) return;

        // Keep the setup portal available while occasionally trying the retained
        // network in STA mode. This lets a temporary router outage self-heal.
        if (!hasCredentials()) return;

        const unsigned long now = millis();
        if (WiFi.status() == WL_CONNECTED)
        {
            systemState.wifiConnected = true;
            apReconnectInProgress = false;

            Serial.println("[WIFI] Saved network restored");
            Serial.println("[WIFI] Returning to normal operation");
            stopAP();
            WiFi.mode(WIFI_STA);
            return;
        }

        if (apReconnectInProgress)
        {
            if (now - apReconnectStartedAt >= RECOVERY_TIMEOUT)
            {
                Serial.println("[WIFI] Saved network still unavailable; provisioning remains active");
                disconnectRadio(false, false);
                apReconnectInProgress = false;
            }
            return;
        }

        if (lastAPReconnectAttempt == 0 || now - lastAPReconnectAttempt >= AP_RECONNECT_INTERVAL)
        {
            lastAPReconnectAttempt = now;
            apReconnectStartedAt = now;
            apReconnectInProgress = true;
            Serial.println("[WIFI] Provisioning active; retrying saved network...");
            WiFi.mode(WIFI_AP_STA);
            WiFi.begin(ssid.c_str(), password.c_str());
        }
        return;
    }

    // ------------------------------------------------------------------
    // STA connection state machine.
    //
    // Exactly one association attempt is ever in flight. While CONNECTING the
    // radio is only polled - WiFi.begin()/WiFi.mode()/WiFi.config() are never
    // re-issued - which is what removes "wifi:sta is connecting, cannot set
    // config". Every state below is non-blocking: it either observes status or
    // starts a single attempt, then returns to loop() so sensing, safety,
    // automation, actuators, schedules, notifications and GSM keep running.
    // ------------------------------------------------------------------

    const unsigned long now = millis();
    const bool linkUp = (WiFi.status() == WL_CONNECTED);

    systemState.wifiConnected = linkUp;

    // ROOT CAUSE of the "Connection lost" / "reconnect handled by ESP32
    // auto-reconnect" storm seen on real hardware with no corresponding
    // [WIFI-EVENT] Disconnected logs: this guard used to only return early
    // when linkUp was true AND the state wasn't already CONNECTED. When the
    // state WAS already CONNECTED, that second clause was always false, so
    // the guard never short-circuited - execution fell straight into
    // switch(wifiState)'s CONNECTED case on every single update() call
    // regardless of linkUp. That case unconditionally treats being reached
    // as "an established link dropped" (see its own comment below), so it
    // printed "Connection lost" and moved to RETRY_WAIT even while still
    // genuinely connected; the very next update() then saw linkUp still
    // true with wifiState now RETRY_WAIT, took the branch below (correctly,
    // for what it actually detects), and printed "reconnect handled by
    // ESP32 auto-reconnect" - a complete, self-inflicted, radio-independent
    // CONNECTED<->RETRY_WAIT oscillation with no real disconnect, no real
    // reconnect, and nothing for the STA event handler to ever report.
    // Steady-state "still connected" must be a plain no-op, never fall into
    // the switch at all - covers a link that came up in any non-CONNECTED
    // state too, including one restored by the provisioning-mode AP+STA
    // retry just before stopAP().
    if (linkUp)
    {
        if (wifiState != WifiState::CONNECTED)
        {
            enterConnectedState();
        }
        return;
    }

    switch (wifiState)
    {
        case WifiState::IDLE:
            // Nothing has been started yet (or provisioning handed control
            // back). Only begin if there is actually something to connect to;
            // a credential-less device is owned by provisioning, not by this.
            if (hasCredentials())
            {
                startConnectionAttempt();
            }
            break;

        case WifiState::CONNECTING:
        {
            // Poll only. No radio reconfiguration happens in this state.
            if (now - connectionStartedAt < RECOVERY_TIMEOUT)
            {
                break;
            }

            Serial.println("[WIFI] Connection attempt timed out");

            // End the failed association before the next attempt. Credentials
            // are held in NVS and in this object; disconnect(false, false)
            // neither erases them nor powers the radio down.
            disconnectRadio(false, false);

            // The outage window was opened by startConnectionAttempt(); this
            // only records that the saved network is now confirmed unusable.
            recoveryInProgress = true;

            logStateChange("CONNECTING", "RETRY_WAIT", "connection attempt timed out");
            wifiState = WifiState::RETRY_WAIT;
            lastReconnectAttempt = now;
            break;
        }

        case WifiState::CONNECTED:
            // linkUp was false to reach here: an established link dropped.
            Serial.println("[WIFI] Connection lost");

            if (recoveryStartedAt == 0)
            {
                recoveryStartedAt = now;
            }
            recoveryInProgress = true;

            logStateChange("CONNECTED", "RETRY_WAIT", "link dropped");
            wifiState = WifiState::RETRY_WAIT;
            lastReconnectAttempt = now;
            Serial.println("[WIFI] Reconnecting in background");
            break;

        case WifiState::RETRY_WAIT:
        {
            // Cumulative, not per-attempt: a saved network that stays
            // unreachable for the whole recovery window hands over to the
            // existing provisioning AP, so a device whose stored SSID/password
            // has gone stale can still be recovered without a re-flash.
            if (recoveryStartedAt != 0 &&
                now - recoveryStartedAt >= RECOVERY_TIMEOUT)
            {
                Serial.println("[WIFI] Saved network unavailable");
                enterAutomaticProvisioningMode();

                // Provisioning now owns the radio; this machine stands down
                // until it hands control back.
                logStateChange("RETRY_WAIT", "IDLE", "recovery window exhausted, handing off to provisioning");
                wifiState = WifiState::IDLE;
                connectionStartedAt = 0;
                recoveryStartedAt = 0;
                break;
            }

            if (now - lastReconnectAttempt >= RECONNECT_INTERVAL)
            {
                Serial.println("[WIFI] Retrying saved network");
                startConnectionAttempt();
            }
            break;
        }
    }
}

void WiFiManager::enterAutomaticProvisioningMode()
{
    if (provisioningMode == ProvisioningMode::MANUAL) return;
    Serial.println("[WIFI] Provisioning mode: FALLBACK");
    if (initialConnectionAttempt)
    {
        Serial.println("[FIREBASE] Not started because Wi-Fi is unavailable");
    }

    // Stop the failed STA association before assigning the radio to the setup AP.
    disconnectRadio(true, false);
    startAP(ProvisioningMode::FALLBACK, !initialConnectionAttempt);
}

void WiFiManager::startManualProvisioning()
{
    if (provisioningMode == ProvisioningMode::MANUAL) return;
    if (isProvisioningMode()) stopAP();
    Serial.println("[WIFI] Provisioning mode: MANUAL");
    startAP(ProvisioningMode::MANUAL, true);
}

void WiFiManager::startAP(ProvisioningMode mode, bool suspendFirebase)
{
    if (isProvisioningMode()) return;

    if (suspendFirebase)
    {
        Serial.println("[FIREBASE] Suspended during provisioning mode");
    }
    Serial.println("[AP] Starting Basilience-Setup");
    WiFi.mode(hasCredentials() ? WIFI_AP_STA : WIFI_AP);
    const IPAddress apIp(192, 168, 4, 1);
    const IPAddress gateway(192, 168, 4, 1);
    const IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(apIp, gateway, subnet);
    if (!WiFi.softAP("Basilience-Setup"))
    {
        Serial.println("[AP] Failed to start Basilience-Setup");
        return;
    }

    delay(500); // Wait for AP to initialize
    
    // Start DNS server to redirect all traffic to ESP32 IP
    dnsServer.start(53, "*", WiFi.softAPIP());
    
    setupAPServer();
    server.begin();

    provisioningMode = mode;
    manualReachabilityLogged = false;
    lastAPReconnectAttempt = millis();
    apReconnectInProgress = false;
    Serial.print("[AP] IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("[AP] HTTP server started");
}

void WiFiManager::stopAP()
{
    if (!isProvisioningMode()) return;
    
    Serial.println("Stopping Access Point...");
    dnsServer.stop();
    server.stop();
    // Keep the STA radio alive when AP+STA recovery has already restored the
    // saved network; only remove the provisioning access point.
    WiFi.softAPdisconnect(false);
    provisioningMode = ProvisioningMode::NONE;
    manualReachabilityLogged = false;
    apReconnectInProgress = false;
}

void WiFiManager::setupAPServer()
{
    server.on("/status", HTTP_GET, [this]() {
        Serial.println("[AP HTTP] GET /status");
        server.send(200, "application/json", "{\"status\":\"setup_mode\"}");
        Serial.println("[AP HTTP] Response: 200");
    });

    server.on("/setup", HTTP_POST, [this]() {
        Serial.println("[AP HTTP] POST /setup");
        if (!server.hasArg("ssid") || !server.hasArg("password")) {
            Serial.println("[AP HTTP] Invalid setup request");
            server.send(400, "text/plain", "Missing SSID or Password");
            Serial.println("[AP HTTP] Response: 400");
            return;
        }
        
        String newSsid = server.arg("ssid");
        String newPass = server.arg("password");

        Serial.print("[AP HTTP] SSID received: ");
        Serial.println(newSsid);
        if (newSsid.isEmpty()) {
            Serial.println("[AP HTTP] Invalid setup request");
            server.send(400, "text/plain", "SSID cannot be empty");
            Serial.println("[AP HTTP] Response: 400");
            return;
        }
        if (!saveCredentials(newSsid, newPass)) {
            server.send(500, "text/plain", "Unable to save credentials");
            Serial.println("[AP HTTP] Response: 500");
            return;
        }
        Serial.println("[AP] Credentials saved");

        // Explicit read-back, independent of the in-memory this->ssid
        // saveCredentials() already set - proves what actually landed in
        // flash rather than assuming the write stuck. Compared directly
        // against the just-submitted value so a divergence (encoding
        // corruption, a write that silently failed) is visible immediately
        // in this same log, instead of only showing up indirectly as a
        // failed reconnect after reboot.
        preferences.begin("wifi", true);
        String verifySsid = preferences.getString("ssid", "");
        preferences.end();
        Serial.print("[AP] NVS read-back ssid=");
        Serial.print(verifySsid);
        Serial.println(verifySsid == newSsid ? " (matches submitted value)" : " (MISMATCH vs submitted value)");

        preferences.begin("wifi", false);
        preferences.putBool("resumeFirebase", true);
        preferences.end();

        server.send(200, "text/plain", "Credentials saved. Connecting device...");
        Serial.println("[AP HTTP] Response: 200");

        if (provisioningMode == ProvisioningMode::MANUAL)
        {
            Serial.println("[WIFI] Manual provisioning completed");
        }

        Serial.println("[WIFI] Reconnecting...");
        delay(1000);
        ESP.restart();
    });

    // Secure Device Auth: one-time migration/provisioning delivery of this
    // device's bootstrap secret. Only ever reachable because this whole HTTP
    // server only exists/serves while provisioning mode is active (started
    // from startAP(), stopped from stopAP()) - there is no separate "enabled"
    // flag to forget, and no path to reach this route during normal
    // operation. The secret is never echoed back and never logged; only its
    // presence/absence is.
    server.on("/secure-provision", HTTP_POST, [this]() {
        Serial.println("[AP HTTP] POST /secure-provision");
        if (!server.hasArg("deviceSecret") || server.arg("deviceSecret").isEmpty()) {
            Serial.println("[AP HTTP] Invalid secure-provision request");
            server.send(400, "text/plain", "Missing deviceSecret");
            Serial.println("[AP HTTP] Response: 400");
            return;
        }

        // generateDeviceSecret.js emits 32 random bytes as unpadded base64url,
        // which is always 43 characters. This bound is intentionally generous
        // (not an exact-length check) so a minor change to the generator's
        // encoding does not brick provisioning, while still rejecting empty-
        // adjacent noise or an oversized payload before it reaches NVS.
        const size_t secretLen = server.arg("deviceSecret").length();
        if (secretLen < 16 || secretLen > 128) {
            Serial.println("[AP HTTP] Invalid secure-provision request (deviceSecret length out of range)");
            server.send(400, "text/plain", "Invalid deviceSecret length");
            Serial.println("[AP HTTP] Response: 400");
            return;
        }

        firebaseManager.saveDeviceSecretFromProvisioning(server.arg("deviceSecret"));

        server.send(200, "text/plain", "Device secret received.");
        Serial.println("[AP HTTP] Response: 200 (secret not logged)");
    });

    // Captive portal redirect for any unknown requests
    server.onNotFound([this]() {
        server.send(200, "text/plain", "Basilience Setup AP active. Connect via the Android App.");
    });
}
