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

    // SIM800L's actual UART baud on this specific board isn't known in
    // advance - unlike the previous module, there's no single confirmed
    // fixed rate for it, and there is no way to ask it (AT+IPR) without
    // already talking to it at its current rate. Rather than hardcode a
    // guess, WAITING_FOR_MODULE cycles the UART through a short list of
    // candidate bauds, giving each MODULE_PROBE_RETRY_INTERVAL_MS to answer
    // "AT" with "OK" before moving to the next - same bounded, non-blocking
    // retry shape this state already used for a single baud, just applied
    // across a small list instead of one fixed value. 115200 (this
    // firmware's existing default) is tried first so an already-correctly-
    // configured module still responds on the very first attempt.
    static constexpr unsigned long BAUD_CANDIDATES[] = {115200UL, 9600UL, 57600UL, 38400UL};
    static constexpr uint8_t BAUD_CANDIDATE_COUNT =
        sizeof(BAUD_CANDIDATES) / sizeof(BAUD_CANDIDATES[0]);
    uint8_t baudCandidateIndex = 0;

    // Every duration below is a bound on how long GsmManager will wait for a
    // given AT response before retrying or giving up - never an indefinite
    // wait. None of them block loop(): update() checks millis() and returns.
    static constexpr unsigned long MODULE_PROBE_RETRY_INTERVAL_MS = 3000UL;
    static constexpr unsigned long SIM_CHECK_RETRY_INTERVAL_MS = 3000UL;
    static constexpr unsigned long REGISTRATION_RETRY_INTERVAL_MS = 5000UL;
    static constexpr unsigned long TEXT_MODE_TIMEOUT_MS = 2000UL;
    static constexpr unsigned long PROMPT_TIMEOUT_MS = 3000UL;
    static constexpr unsigned long SEND_RESULT_TIMEOUT_MS = 15000UL;
    static constexpr size_t RX_BUFFER_CAP = 512;

    void drainSerial();
    void sendCommand(const char* command);
    void beginSendStage(SendStage stage);
    void finishSend(SendResult result);
    void beginSerialAtCurrentBaudCandidate();

    void updateWaitingForModule(unsigned long now);
    void updateCheckingSim(unsigned long now);
    void updateCheckingRegistration(unsigned long now);
    void updateSendingSms(unsigned long now);
};

#endif
