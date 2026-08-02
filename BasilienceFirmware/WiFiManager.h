#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>

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

    bool updateCredentialsSafely(
        const String& newSsid,
        const String& newPassword);

    void clearCredentials();

    bool hasCredentials() const;

    bool isConnected() const;

    String getSSID() const;

    void startAP();
    void stopAP();

private:
    bool loadCredentials();
    void setupAPServer();

    Preferences preferences;
    String ssid;
    String password;

    unsigned long lastReconnectAttempt = 0;
    static constexpr unsigned long RECONNECT_INTERVAL = 10000;

    bool isAPMode = false;
    WebServer server{80};
    DNSServer dnsServer;
};

#endif