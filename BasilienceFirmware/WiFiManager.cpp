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