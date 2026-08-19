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
    enum class ProvisioningMode
    {
        NONE,
        FALLBACK,
        MANUAL
    };

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
    ProvisioningMode getProvisioningMode() const;
    bool consumeFirebaseResumePending();

    String getSSID() const;

    void startManualProvisioning();
    void stopAP();

private:
    bool loadCredentials();
    void setupAPServer();
    void enterAutomaticProvisioningMode();
    void startAP(ProvisioningMode mode, bool suspendFirebase);

    Preferences preferences;
    String ssid;
    String password;

    unsigned long lastReconnectAttempt = 0;
    unsigned long lastAPReconnectAttempt = 0;
    unsigned long apReconnectStartedAt = 0;
    static constexpr unsigned long RECONNECT_INTERVAL = 5000;
    static constexpr unsigned long RECOVERY_TIMEOUT = 20000;
    static constexpr unsigned long AP_RECONNECT_INTERVAL = 30000;

    ProvisioningMode provisioningMode = ProvisioningMode::NONE;
    bool manualReachabilityLogged = false;
    bool recoveryInProgress = false;
    bool apReconnectInProgress = false;
    bool firebaseResumePending = false;
    bool initialConnectionAttempt = true;
    WebServer server{80};
    DNSServer dnsServer;
};

#endif
