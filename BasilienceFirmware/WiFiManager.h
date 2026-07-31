#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>

class WiFiManager
{
public:


    void begin();

    bool connect();
    void update();

    void disconnect();

    bool reconnect();

    bool saveCredentials(
        const String& ssid,
        const String& password);

    void clearCredentials();

    bool hasCredentials() const;

    bool isConnected() const;

    String getSSID() const;

private:
    bool loadCredentials();

        Preferences preferences;

        String ssid;

        String password;

        unsigned long lastReconnectAttempt = 0;

        static constexpr unsigned long
            RECONNECT_INTERVAL = 10000;
};

#endif