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

    // Explicit STA connection state. Previously this was implied by a pair of
    // booleans, which made it possible to re-issue WiFi.begin()/WiFi.mode()
    // while a previous association attempt was still in flight - the condition
    // that produces "wifi:sta is connecting, cannot set config". With the state
    // named, CONNECTING is a period during which the radio is only polled.
    enum class WifiState
    {
        IDLE,
        CONNECTING,
        CONNECTED,
        RETRY_WAIT
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

    // The single saved-credential STA initiation path. Nothing else in the
    // normal retry loop may call WiFi.begin()/WiFi.mode().
    void startConnectionAttempt();

    // Shared CONNECTED transition: clears the cumulative recovery window and
    // logs the connection once.
    void enterConnectedState();

    Preferences preferences;
    String ssid;
    String password;

    WifiState wifiState = WifiState::IDLE;

    // Start of the current single association attempt - drives the per-attempt
    // timeout only.
    unsigned long connectionStartedAt = 0;

    // Start of the current *cumulative* outage window. Set once when the saved
    // network is first confirmed unusable and deliberately NOT reset by each
    // individual 5-second retry, so the fallback timeout measures the whole
    // outage rather than one attempt. Cleared only by a successful connection.
    unsigned long recoveryStartedAt = 0;

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
