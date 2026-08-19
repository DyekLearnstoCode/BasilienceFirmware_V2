#ifndef GSM_RAW_UART_TEST_H
#define GSM_RAW_UART_TEST_H

#include <Arduino.h>

// TEMPORARY hardware bring-up diagnostic only - gated entirely behind
// GSM_RAW_UART_TEST in Config.h (default false). Bypasses GsmManager's
// parser/state machine completely: runs a deterministic, one-shot AT
// command sequence (AT / ATE0 / ATI / AT+CPIN? / AT+CSQ / AT+CEREG? /
// AT+CREG? / AT+COPS?, then an optional SMS send) equivalent to the
// hardware-UART test already proven working against this same A7680C on
// ESP8266, and prints every command and response.
//
// Never wired into the notification pipeline, never touches the SMS
// recipient cache, never uses production SMS architecture - the test
// message destination is GSM_TEST_PHONE_NUMBER (Config.h), a temporary
// diagnostic-only constant. Non-blocking (millis()-driven, no delay()) so
// the cultivation loop is unaffected whichever mode is active. Runs once to
// a finished state and then stops touching the UART entirely. When
// GSM_RAW_UART_TEST is false, this class is simply never constructed-into-
// use - production behavior is unchanged.
class GsmRawUartTest
{
public:
    void begin();
    void update();

private:
    enum class Step : uint8_t
    {
        AT_PROBE,
        ATE0,
        ATI,
        CPIN,
        CSQ,
        CEREG,
        CREG,
        COPS,
        SMS_DECISION,
        SMS_CMGF,
        SMS_CMGS_CMD,
        SMS_BODY,
        FINISHED
    };

    enum class SmsResult : uint8_t { NOT_ATTEMPTED, PASS, FAIL };

    static constexpr unsigned long AT_ATTEMPT_TIMEOUT_MS = 2000UL;
    static constexpr uint8_t AT_MAX_ATTEMPTS = 5;
    static constexpr unsigned long STANDARD_TIMEOUT_MS = 5000UL;
    static constexpr unsigned long SMS_PROMPT_TIMEOUT_MS = 10000UL;
    static constexpr unsigned long SMS_SEND_TIMEOUT_MS = 30000UL;
    static constexpr size_t RESPONSE_BUFFER_CAP = 256;

    HardwareSerial serial{1};
    Step step = Step::AT_PROBE;
    uint8_t atAttempt = 0;
    unsigned long stepStartedAt = 0;
    bool commandSentThisStep = false;
    bool finalResultPrinted = false;

    char responseBuffer[RESPONSE_BUFFER_CAP + 1] = {0};
    size_t responseLen = 0;

    // Carried across steps to gate SMS eligibility and print the final
    // PASS/FAIL/NOT ATTEMPTED summary.
    bool uartPass = false;
    bool simReady = false;
    bool networkRegistered = false;
    SmsResult smsResult = SmsResult::NOT_ATTEMPTED;
    const char* smsSkipReason = nullptr;

    void sendCommand(const char* command);
    void resetResponseBuffer();
    void appendIncomingBytes();
    bool responseContains(const char* marker) const;
    bool parseRegistrationStatus(const char* prefix, bool& registeredOut) const;

    // Drains available bytes into responseBuffer and returns true once
    // either OK/ERROR (which also covers "+CME ERROR"/"+CMS ERROR", both
    // containing the substring "ERROR") appears, or timeoutMs has elapsed
    // since stepStartedAt - the caller inspects the buffer either way.
    bool pollForTerminator(unsigned long timeoutMs);

    void transitionTo(Step next);
    void printStepResponse() const;
    void printFinalResult();

    void handleAtProbe();
    void handleSimpleQuery(const char* command, Step next);
    void handleCpin();
    void handleCereg();
    void handleCreg();
    void handleSmsDecision();
    void handleSmsCmgf();
    void handleSmsCmgsCommand();
    void handleSmsBody();
};

#endif
