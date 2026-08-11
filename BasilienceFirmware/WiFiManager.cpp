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

    connect();
    initialConnectionAttempt = false;
}


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

    systemState.wifiConnected =
        WiFi.status() == WL_CONNECTED;

    if (systemState.wifiConnected)
    {
        if (recoveryInProgress)
        {
            Serial.println("[WIFI] Saved network restored");
            Serial.println("[WIFI] Returning to normal operation");
            recoveryInProgress = false;
        }
        return;
    }

    const unsigned long now = millis();
    if (!recoveryInProgress)
    {
        recoveryInProgress = true;
        recoveryStartedAt = now;
        lastReconnectAttempt = 0;
        Serial.println("[WIFI] Connection lost");
        Serial.println("[WIFI] Attempting reconnection...");
    }

    if (now - recoveryStartedAt >= RECOVERY_TIMEOUT)
    {
        Serial.println("[WIFI] Reconnection timeout");
        Serial.println("[WIFI] Starting provisioning AP");
        Serial.println("[WIFI] Saved credentials retained");
        recoveryInProgress = false;
        enterAutomaticProvisioningMode();
        return;
    }

    if (lastReconnectAttempt != 0 && now - lastReconnectAttempt < RECONNECT_INTERVAL)
    {
        return;
    }

    lastReconnectAttempt = now;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
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

    // Captive portal redirect for any unknown requests
    server.onNotFound([this]() {
        server.send(200, "text/plain", "Basilience Setup AP active. Connect via the Android App.");
    });
}
