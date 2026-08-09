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
    bool isProvisioningMode() const;
    bool consumeFirebaseResumePending();

    String getSSID() const;

    void startAP(bool suspendFirebase = true);
    void stopAP();

private:
    bool loadCredentials();
    void setupAPServer();
    void enterAutomaticProvisioningMode();

    Preferences preferences;
    String ssid;
    String password;

    unsigned long lastReconnectAttempt = 0;
    static constexpr unsigned long RECONNECT_INTERVAL = 10000;

    bool isAPMode = false;
    bool firebaseResumePending = false;
    bool initialConnectionAttempt = true;
    WebServer server{80};
    DNSServer dnsServer;
};

#endif
