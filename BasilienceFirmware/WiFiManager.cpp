#include "WiFiManager.h"

#include "Globals.h"

void WiFiManager::begin()
{
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

    Serial.print("[WIFI] Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());

    connectionStartedAt = millis();
    lastReconnectAttempt = connectionStartedAt;
    wifiState = WifiState::CONNECTING;

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
    WiFi.disconnect();

    systemState.wifiConnected = false;

    Serial.println("WiFi Disconnected");
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

    preferences.putString("ssid", ssid);

    preferences.putString("password", password);

    preferences.end();

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

        // Keep the setup portal available while occasionally trying the retained
        // network in STA mode. This lets a temporary router outage self-heal.
        if (!hasCredentials()) return;

        const unsigned long now = millis();
        if (WiFi.status() == WL_CONNECTED)
        {
            systemState.wifiConnected = true;
            apReconnectInProgress = false;
            if (provisioningMode == ProvisioningMode::MANUAL)
            {
                if (!manualReachabilityLogged)
                {
                    Serial.println("[WIFI] Saved network still reachable");
                    Serial.println("[WIFI] Manual provisioning active - keeping setup AP open");
                    manualReachabilityLogged = true;
                }
                return;
            }

            Serial.println("[WIFI] Saved network restored");
            Serial.println("[WIFI] Returning to normal operation");
            stopAP();
            WiFi.mode(WIFI_STA);
            return;
        }

        manualReachabilityLogged = false;

        if (apReconnectInProgress)
        {
            if (now - apReconnectStartedAt >= RECOVERY_TIMEOUT)
            {
                Serial.println("[WIFI] Saved network still unavailable; provisioning remains active");
                WiFi.disconnect(false, false);
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

    // Covers a link that came up in any non-CONNECTED state, including one
    // restored by the provisioning-mode AP+STA retry just before stopAP().
    if (linkUp && wifiState != WifiState::CONNECTED)
    {
        enterConnectedState();
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
            WiFi.disconnect(false, false);

            // The outage window was opened by startConnectionAttempt(); this
            // only records that the saved network is now confirmed unusable.
            recoveryInProgress = true;

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
    WiFi.disconnect(true, false);
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
