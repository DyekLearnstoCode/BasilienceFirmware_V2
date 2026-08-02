#include "WiFiManager.h"

#include "Config.h"
#include "Globals.h"

void WiFiManager::begin()
{
    if (!loadCredentials())
    {
        Serial.println("No saved WiFi credentials.");

        saveCredentials(
            WIFI_SSID,
            WIFI_PASSWORD);

        loadCredentials();
    }

    connect();
}


bool WiFiManager::connect()
{
    if (!hasCredentials())
    {
        Serial.println("No WiFi credentials available.");

        systemState.wifiConnected = false;

        return false;
    }

    // Already connected
    if (WiFi.status() == WL_CONNECTED)
    {
        systemState.wifiConnected = true;

        return true;
    }

    Serial.println();
    Serial.println("Connecting WiFi...");

    WiFi.mode(WIFI_STA);

    WiFi.begin(
        ssid.c_str(),
        password.c_str());

    unsigned long startTime = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - startTime >= 30000)
        {
            Serial.println();
            Serial.println("WiFi Connection Timeout");

            systemState.wifiConnected = false;
            startAP(); // Start AP mode for in-app configuration

            return false;
        }

        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi Connected");

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
    return !ssid.isEmpty() &&
           !password.isEmpty();
}

bool WiFiManager::isConnected() const
{
    return WiFi.status() == WL_CONNECTED;
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
    if (isAPMode)
    {
        dnsServer.processNextRequest();
        server.handleClient();
        return;
    }

    systemState.wifiConnected =
        WiFi.status() == WL_CONNECTED;

    if (systemState.wifiConnected)
    {
        return;
    }

    if (millis() - lastReconnectAttempt <
        RECONNECT_INTERVAL)
    {
        return;
    }

    lastReconnectAttempt = millis();

    Serial.println();
    Serial.println("WiFi disconnected.");

    Serial.println("Attempting reconnection...");

    reconnect();
}

void WiFiManager::startAP()
{
    if (isAPMode) return;
    
    Serial.println("Starting Access Point: Basilience-Setup");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Basilience-Setup"); // Open network for easy setup

    delay(500); // Wait for AP to initialize
    
    // Start DNS server to redirect all traffic to ESP32 IP
    dnsServer.start(53, "*", WiFi.softAPIP());
    
    setupAPServer();
    server.begin();
    
    isAPMode = true;
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
}

void WiFiManager::stopAP()
{
    if (!isAPMode) return;
    
    Serial.println("Stopping Access Point...");
    dnsServer.stop();
    server.stop();
    WiFi.softAPdisconnect(true);
    isAPMode = false;
}

void WiFiManager::setupAPServer()
{
    server.on("/status", HTTP_GET, [this]() {
        server.send(200, "application/json", "{\"status\":\"setup_mode\"}");
    });

    server.on("/setup", HTTP_POST, [this]() {
        if (!server.hasArg("ssid") || !server.hasArg("password")) {
            server.send(400, "text/plain", "Missing SSID or Password");
            return;
        }
        
        String newSsid = server.arg("ssid");
        String newPass = server.arg("password");
        
        Serial.println("Received new WiFi credentials from App.");
        server.send(200, "text/plain", "Credentials received. Rebooting...");
        
        saveCredentials(newSsid, newPass);
        
        delay(1000);
        ESP.restart(); // Safest way to apply new WiFi credentials cleanly
    });

    // Captive portal redirect for any unknown requests
    server.onNotFound([this]() {
        server.send(200, "text/plain", "Basilience Setup AP active. Connect via the Android App.");
    });
}