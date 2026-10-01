#ifndef GSM_MANAGER_H
#define GSM_MANAGER_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// Foundation driver for a SIMCom SIM800L GSM/GPRS module. Owns the dedicated
// GSM UART and a millis()-driven state machine so cultivation control
// (sensors/automation/safety/actuators) is never blocked while the module
// boots or registers - update() never calls delay() or spins in a wait loop
// for those phases. SENDING AN SMS IS THE ONE EXCEPTION: see `serial`'s own
// declaration-site comment below - actually writing the AT+CMGS body is a
// bounded but genuinely blocking call, not non-blocking like everything
// else here. GsmManager knows nothing about Firebase, user roles, or
// alert/delivery policy: it only sends text a caller supplies to a number a
// caller supplies, one at a time, and reports why if it couldn't.
//
// The AT/SMS command set here (AT, AT+CPIN?, AT+CMGF=1, AT+CMGS) is standard
// Hayes/3GPP TS 27.005 and behaves identically across GSM modules; the one
// exception was the registration query, which used to be AT+CEREG? (EPS/LTE
// registration - the previous target module, an A76XX/A7680C, is LTE Cat1).
// SIM800L is 2G/GPRS-only and has no EPS stack, so registration is now
// checked with the legacy circuit-switched AT+CREG? instead - the response
// shape (+CREG: <n>,<stat>) and status codes match CEREG's, so the parsing
// logic needed no change beyond the command/tag string.
//
// Hardened against the physically bench-validated SIM800L V2 unit (see the
// GSM physical validation report): READY now periodically re-verifies
// registration rather than trusting it forever (a lost registration used to
// be invisible until a send silently hung), and an unsolicited "RDY" line -
// SIM800L's own signature for "I just (re)booted" - is recognized from any
// state so a genuine modem restart (e.g. a power blip) cleanly aborts any
// in-flight send and re-enters initialization instead of leaving the state
// machine wedged in a stale READY.
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
    // normalize) if the module is READY and idle. Returns true if accepted
    // and now in progress; false if rejected immediately, in which case
    // getLastResult() reports why. Either way this call never blocks - a
    // caller feeding multiple recipients should poll isBusy()/getLastResult()
    // each loop() and call sendSms() again once the previous one finishes.
    bool sendSms(const String& phoneNumber, const String& message);

    bool isBusy() const;
    bool isReady() const;
    State getState() const;
    SendResult getLastResult() const;

    // Human-readable form of the current state - diagnostic only (e.g.
    // NotificationManager's "[SMS] Waiting for GSM" log when a queued SMS
    // can't start yet), mirrors WiFiManager::stateName()'s own precedent.
    // Never read by anything that affects control flow.
    const char* stateName() const;

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

    // UART investigation (GSM send-path audit): switched from
    // HardwareSerial{1} (UART1 via the ESP32 GPIO matrix) to
    // EspSoftwareSerial - a proven-working standalone reference sketch
    // (GsmSimTest.ino, repo root) using SoftwareSerial on these same
    // GSM_RX_PIN/GSM_TX_PIN communicated with this module correctly, while
    // [GSM-UART] diagnostics confirmed HardwareSerial received zero bytes
    // ever on GPIO36 (a no-internal-pull-up, input-only ESP32 pin) despite
    // identical wiring/baud. Stream-compatible (extends Stream, same as
    // HardwareSerial) - every existing available()/read()/print()/
    // println() call below is unaffected; only this declaration and
    // beginSerial()'s begin() call change.
    //
    // One real behavioral difference: unlike HardwareSerial (FIFO +
    // interrupt-driven, non-blocking on write), SoftwareSerial's write()
    // bit-bangs each byte with busy-wait timing and briefly disables
    // interrupts for the whole call - see beginSerial()'s and the class
    // comment's own notes. RX stays interrupt-driven/non-blocking exactly
    // like before.
    SoftwareSerial serial;

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

    // Every duration below bounds how long GsmManager waits for a given AT
    // response before retrying or giving up - never indefinite. None of
    // them block loop(): update() checks millis() and returns.
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

    // GSM send-path audit (UART investigation): counts every "AT" module
    // probe sent from begin()/updateWaitingForModule()'s retry/
    // checkForModemRestart()'s reinit - NOT the AT+CPIN?/AT+CREG? queries
    // in later states, which have no diagnostic gap to fill. Monotonic for
    // the whole session (not reset on a successful transition out of
    // WAITING_FOR_MODULE) so "attempt=N" in the log directly says how many
    // probes preceded whatever happened next, across any later restart
    // episode too.
    uint16_t atProbeAttempts = 0;
    void logAtProbeAttempt();

    // Bounded, printable-only rendering of a raw UART buffer for the
    // [GSM-UART] diagnostics - replaces any byte outside printable ASCII
    // (CR/LF included) with '.' and truncates to maxLen, so a garbled/
    // binary response is still safely loggable and never floods the
    // Serial Monitor. Contains only modem protocol text (AT responses),
    // never phone numbers or message bodies - nothing here needs masking
    // for that reason, only for length/printability.
    static String sanitizeForLog(const String& raw, size_t maxLen);

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
