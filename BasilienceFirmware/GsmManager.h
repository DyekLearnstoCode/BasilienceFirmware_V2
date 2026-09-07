#ifndef GSM_MANAGER_H
#define GSM_MANAGER_H

#include <Arduino.h>

// Foundation driver for a SIMCom SIM800L GSM/GPRS module. Owns the
// dedicated GSM UART and a millis()-driven state machine so cultivation
// control (sensors/automation/safety/actuators) is never blocked while the
// module boots, registers, or sends an SMS - update() never calls delay() or
// spins in a wait loop. GsmManager knows nothing about Firebase, user roles,
// or alert/delivery policy: it only sends text a caller supplies to a number
// a caller supplies, one at a time, and reports why if it couldn't.
//
// The AT/SMS command set here (AT, AT+CPIN?, AT+CMGF=1, AT+CMGS) is standard
// Hayes/3GPP TS 27.005 and behaves identically across GSM modules; the one
// exception was the registration query, which used to be AT+CEREG? (EPS/LTE
// registration - the previous target module, an A76XX/A7680C, is LTE Cat1).
// SIM800L is 2G/GPRS-only and has no EPS stack, so registration is now
// checked with the legacy circuit-switched AT+CREG? instead - the response
// shape (+CREG: <n>,<stat>) and status codes are the same as CEREG's, so the
// parsing logic needed no change beyond the command/tag string itself.
//
// Hardened against the physically bench-validated SIM800L V2 unit (see the
// GSM physical validation report): READY now periodically re-verifies
// registration rather than trusting it forever (a lost registration used to
// be invisible until a send silently hung), and an unsolicited "RDY" line -
// SIM800L's own signature for "I just (re)booted" - is recognized from any
// state so a genuine modem restart (e.g. a power blip) cleanly aborts any
// in-flight send and re-enters initialization instead of leaving the state
// machine wedged in a READY that no longer reflects reality.
class GsmManager
{
public:
    enum class State : uint8_t
    {
        WAITING_FOR_MODULE,
        CHECKING_SIM,
        CHECKING_REGISTRATION,
        READY,
        SENDING_SMS
    };

    enum class SendResult : uint8_t
    {
        NONE,
        SUCCESS,
        MODULE_NOT_READY,
        SIM_NOT_READY,
        NOT_REGISTERED,
        INVALID_NUMBER,
        BUSY,
        TIMEOUT,
        ERROR
    };

    void begin();
    void update();

    // Starts sending `message` to `phoneNumber` (must already be in canonical
    // +639XXXXXXXXX form - this class validates defensively but does not
    // normalize) if the module is READY and idle. Returns true if the
    // request was accepted and is now in progress; false if rejected
    // immediately, in which case getLastResult() reports why. Either way this
    // call itself never blocks - a caller feeding multiple recipients should
    // poll isBusy()/getLastResult() each loop() and call sendSms() again for
    // the next recipient once the previous one finishes.
    bool sendSms(const String& phoneNumber, const String& message);

    bool isBusy() const;
    bool isReady() const;
    State getState() const;
    SendResult getLastResult() const;

    // Structural-only validation of the canonical +639XXXXXXXXX form (13
    // chars: '+', "63", "9", then 9 more digits). No carrier-prefix table -
    // matches the Android-side PhoneNumberUtils normalization contract.
    static bool isValidCanonicalPhilippineMobile(const String& phoneNumber);

    // Masked form for diagnostics/logging, e.g. "+63917****567". Public so
    // other components (e.g. NotificationManager) can mask numbers in their
    // own serial logs without duplicating this logic.
    static String maskPhoneNumber(const String& phoneNumber);

private:
    enum class SendStage : uint8_t
    {
        NONE,
        SET_TEXT_MODE,
        AWAIT_PROMPT,
        AWAIT_SEND_RESULT
    };

    HardwareSerial serial{1};

    State state = State::WAITING_FOR_MODULE;
    SendResult lastResult = SendResult::NONE;
    SendStage sendStage = SendStage::NONE;

    String rxBuffer;
    String pendingNumber;
    String pendingMessage;

    unsigned long stageStartedAt = 0;

    // Timestamp of the last confirmed-registered CREG check (set both on
    // the initial CHECKING_REGISTRATION -> READY transition and on every
    // periodic re-check from READY). Compared against
    // REGISTRATION_HEALTH_INTERVAL_MS to decide when READY is due for
    // another look - see updateReady().
    unsigned long lastRegistrationCheckAt = 0;

    // Every duration below is a bound on how long GsmManager will wait for a
    // given AT response before retrying or giving up - never an indefinite
    // wait. None of them block loop(): update() checks millis() and returns.
    static constexpr unsigned long MODULE_PROBE_RETRY_INTERVAL_MS = 3000UL;
    static constexpr unsigned long SIM_CHECK_RETRY_INTERVAL_MS = 3000UL;
    static constexpr unsigned long REGISTRATION_RETRY_INTERVAL_MS = 5000UL;
    // Bounded health poll: how often READY re-issues AT+CREG? to catch a
    // registration loss that happens after the initial connect (previously
    // unmonitored - see the class-level comment). Conservative on purpose -
    // this is a liveness check, not a diagnostic feed, and must not spam
    // the modem or contend with an in-flight send.
    static constexpr unsigned long REGISTRATION_HEALTH_INTERVAL_MS = 60000UL;
    static constexpr unsigned long TEXT_MODE_TIMEOUT_MS = 2000UL;
    static constexpr unsigned long PROMPT_TIMEOUT_MS = 3000UL;
    // Bench sends completed quickly, but real SMSC submission over the air
    // can legitimately take longer than that under load - and because this
    // is purely a state-machine bound (update() never blocks loop() while
    // waiting it out), there is no cost to giving it generous room before
    // declaring TIMEOUT.
    static constexpr unsigned long SEND_RESULT_TIMEOUT_MS = 45000UL;
    static constexpr size_t RX_BUFFER_CAP = 512;

    void drainSerial();
    void sendCommand(const char* command);
    void beginSendStage(SendStage stage);
    void finishSend(SendResult result);
    void beginSerial();
    void logSendError(const String& response) const;

    // Detects an unsolicited "RDY" anywhere in rxBuffer - SIM800L's own
    // signal that it just (re)booted - from any state other than
    // WAITING_FOR_MODULE (where it's expected/harmless noise ahead of the
    // next "AT" probe). Aborts any in-flight send and resets the state
    // machine back to module initialization. Returns true if it acted, in
    // which case update() must not also run this tick's normal state
    // handler against now-stale state.
    bool checkForModemRestart(unsigned long now);

    void updateWaitingForModule(unsigned long now);
    void updateCheckingSim(unsigned long now);
    void updateCheckingRegistration(unsigned long now);
    void updateReady(unsigned long now);
    void updateSendingSms(unsigned long now);
};

#endif
