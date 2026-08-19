#include "GsmRawUartTest.h"
#include "Config.h"

namespace
{
    const char* const SMS_TEST_MESSAGE_BODY = "Basilience A7680C ESP32 SMS test";
}

void GsmRawUartTest::begin()
{
    Serial.println("================================");
    Serial.println(" A7680C ESP32 FULL DIAGNOSTIC");
    Serial.println("================================");
    Serial.print("RX GPIO: ");
    Serial.println(GSM_RX_PIN);
    Serial.print("TX GPIO: ");
    Serial.println(GSM_TX_PIN);
    Serial.print("Baud: ");
    Serial.println(GSM_BAUD_RATE);
    // PWRK is not currently wired to an ESP32 GPIO - this diagnostic cannot
    // pulse it. Automatic PWRK control is a separate, later task.
    Serial.println("[GSM-TEST] Ensure A7680C is powered and PWRK has been activated.");
    Serial.println();

    serial.begin(GSM_BAUD_RATE, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    while (serial.available() > 0) serial.read(); // discard boot-time noise

    step = Step::AT_PROBE;
    atAttempt = 0;
    finalResultPrinted = false;
    uartPass = false;
    simReady = false;
    networkRegistered = false;
    smsResult = SmsResult::NOT_ATTEMPTED;
    smsSkipReason = nullptr;
    commandSentThisStep = false;
    resetResponseBuffer();
}

void GsmRawUartTest::update()
{
    if (step == Step::FINISHED)
    {
        if (!finalResultPrinted)
        {
            printFinalResult();
            finalResultPrinted = true;
        }
        // Fully idle from here on - no UART reads, no further prints, per
        // spec: the diagnostic must not consume/print random bytes forever.
        return;
    }

    switch (step)
    {
        case Step::AT_PROBE:
            handleAtProbe();
            break;
        case Step::ATE0:
            handleSimpleQuery("ATE0", Step::ATI);
            break;
        case Step::ATI:
            handleSimpleQuery("ATI", Step::CPIN);
            break;
        case Step::CPIN:
            handleCpin();
            break;
        case Step::CSQ:
            handleSimpleQuery("AT+CSQ", Step::CEREG);
            break;
        case Step::CEREG:
            handleCereg();
            break;
        case Step::CREG:
            handleCreg();
            break;
        case Step::COPS:
            handleSimpleQuery("AT+COPS?", Step::SMS_DECISION);
            break;
        case Step::SMS_DECISION:
            handleSmsDecision();
            break;
        case Step::SMS_CMGF:
            handleSmsCmgf();
            break;
        case Step::SMS_CMGS_CMD:
            handleSmsCmgsCommand();
            break;
        case Step::SMS_BODY:
            handleSmsBody();
            break;
        case Step::FINISHED:
            break; // handled above
    }
}

void GsmRawUartTest::sendCommand(const char* command)
{
    resetResponseBuffer();
    serial.print(command);
    serial.print("\r\n");
    Serial.print("[GSM-TEST] >> ");
    Serial.println(command);
    commandSentThisStep = true;
    stepStartedAt = millis();
}

void GsmRawUartTest::resetResponseBuffer()
{
    responseLen = 0;
    responseBuffer[0] = '\0';
}

void GsmRawUartTest::appendIncomingBytes()
{
    // Only printable ASCII plus CR/LF enters the buffer - matches the old
    // raw-scan diagnostic's filtering, so stray line noise never corrupts a
    // response string this code goes on to strstr()/sscanf().
    while (serial.available() > 0 && responseLen < RESPONSE_BUFFER_CAP)
    {
        char c = (char)serial.read();
        if (c == '\r' || c == '\n' || (c >= 0x20 && c <= 0x7E))
        {
            responseBuffer[responseLen++] = c;
        }
    }
    responseBuffer[responseLen] = '\0';
}

bool GsmRawUartTest::responseContains(const char* marker) const
{
    return strstr(responseBuffer, marker) != nullptr;
}

bool GsmRawUartTest::pollForTerminator(unsigned long timeoutMs)
{
    appendIncomingBytes();

    // "ERROR" as a substring also covers "+CME ERROR" and "+CMS ERROR".
    if (responseContains("OK") || responseContains("ERROR")) return true;
    if (millis() - stepStartedAt >= timeoutMs) return true;
    return false;
}

bool GsmRawUartTest::parseRegistrationStatus(const char* prefix, bool& registeredOut) const
{
    const char* found = strstr(responseBuffer, prefix);
    if (found == nullptr) return false;

    int mode = -1;
    int stat = -1;
    if (sscanf(found + strlen(prefix), "%d,%d", &mode, &stat) != 2) return false;

    // 1 = registered, home network. 5 = registered, roaming.
    registeredOut = (stat == 1 || stat == 5);
    return true;
}

void GsmRawUartTest::transitionTo(Step next)
{
    step = next;
    commandSentThisStep = false;
    resetResponseBuffer();
    stepStartedAt = millis();
}

void GsmRawUartTest::printStepResponse() const
{
    if (responseLen == 0) return;
    Serial.print(responseBuffer);
    if (responseBuffer[responseLen - 1] != '\n') Serial.println();
}

void GsmRawUartTest::printFinalResult()
{
    Serial.println();
    Serial.println("================================");
    Serial.println(" FINAL RESULT");
    Serial.println("================================");
    Serial.print("UART    : ");
    Serial.println(uartPass ? "PASS" : "FAIL");
    Serial.print("SIM     : ");
    Serial.println(simReady ? "PASS" : "FAIL");
    Serial.print("NETWORK : ");
    Serial.println(networkRegistered ? "PASS" : "FAIL");
    Serial.print("SMS     : ");
    switch (smsResult)
    {
        case SmsResult::PASS:
            Serial.println("PASS");
            break;
        case SmsResult::FAIL:
            Serial.print("FAIL");
            if (smsSkipReason != nullptr)
            {
                Serial.print(" (");
                Serial.print(smsSkipReason);
                Serial.print(")");
            }
            Serial.println();
            break;
        case SmsResult::NOT_ATTEMPTED:
            Serial.println("NOT ATTEMPTED");
            break;
    }
    Serial.println("================================");
    Serial.println("[GSM-TEST] Diagnostic finished.");
}

void GsmRawUartTest::handleAtProbe()
{
    if (!commandSentThisStep)
    {
        atAttempt++;
        Serial.print("[GSM-TEST] Attempt ");
        Serial.print(atAttempt);
        Serial.print("/");
        Serial.println(AT_MAX_ATTEMPTS);
        sendCommand("AT");
        return;
    }

    if (!pollForTerminator(AT_ATTEMPT_TIMEOUT_MS)) return;

    printStepResponse();

    if (responseContains("OK"))
    {
        uartPass = true;
        Serial.println("[PASS] Basic AT communication");
        transitionTo(Step::ATE0);
        return;
    }

    if (atAttempt >= AT_MAX_ATTEMPTS)
    {
        uartPass = false;
        Serial.println("[FAIL] Basic AT communication - CONNECTION FAIL");
        transitionTo(Step::FINISHED);
        return;
    }

    // Retry on the next update() call - the timeout window just elapsed
    // already provides the "reasonable delay" between attempts.
    commandSentThisStep = false;
}

void GsmRawUartTest::handleSimpleQuery(const char* command, Step next)
{
    if (!commandSentThisStep)
    {
        sendCommand(command);
        return;
    }

    if (!pollForTerminator(STANDARD_TIMEOUT_MS)) return;

    printStepResponse();
    transitionTo(next);
}

void GsmRawUartTest::handleCpin()
{
    if (!commandSentThisStep)
    {
        sendCommand("AT+CPIN?");
        return;
    }

    if (!pollForTerminator(STANDARD_TIMEOUT_MS)) return;

    printStepResponse();

    // Recognizes +CPIN: READY as ready; +CPIN: SIM PIN/SIM PUK and every
    // +CME ERROR variant (e.g. "SIM not inserted") as not ready. The raw
    // response is always printed above regardless, so the specific reason
    // is never hidden even though simReady itself is a simple boolean.
    simReady = responseContains("READY");
    Serial.println(simReady ? "[PASS] SIM READY" : "[FAIL] SIM NOT READY");

    transitionTo(Step::CSQ);
}

void GsmRawUartTest::handleCereg()
{
    if (!commandSentThisStep)
    {
        sendCommand("AT+CEREG?");
        return;
    }

    if (!pollForTerminator(STANDARD_TIMEOUT_MS)) return;

    printStepResponse();

    bool registered = false;
    if (parseRegistrationStatus("+CEREG:", registered) && registered)
    {
        networkRegistered = true;
    }

    transitionTo(Step::CREG);
}

void GsmRawUartTest::handleCreg()
{
    if (!commandSentThisStep)
    {
        sendCommand("AT+CREG?");
        return;
    }

    if (!pollForTerminator(STANDARD_TIMEOUT_MS)) return;

    printStepResponse();

    bool registered = false;
    if (parseRegistrationStatus("+CREG:", registered) && registered)
    {
        networkRegistered = true;
    }

    transitionTo(Step::COPS);
}

void GsmRawUartTest::handleSmsDecision()
{
    Serial.println();

    if (!simReady)
    {
        smsSkipReason = "SIM not ready";
        Serial.println("SMS NOT ATTEMPTED");
        Serial.println("Reason: SIM not ready");
        transitionTo(Step::FINISHED);
        return;
    }

    if (!networkRegistered)
    {
        smsSkipReason = "network not registered";
        Serial.println("SMS NOT ATTEMPTED");
        Serial.println("Reason: network not registered");
        transitionTo(Step::FINISHED);
        return;
    }

    transitionTo(Step::SMS_CMGF);
}

void GsmRawUartTest::handleSmsCmgf()
{
    if (!commandSentThisStep)
    {
        sendCommand("AT+CMGF=1");
        return;
    }

    if (!pollForTerminator(STANDARD_TIMEOUT_MS)) return;

    printStepResponse();

    if (!responseContains("OK"))
    {
        smsResult = SmsResult::FAIL;
        smsSkipReason = "AT+CMGF=1 failed";
        Serial.println("[SMS FAIL] Could not set text mode");
        transitionTo(Step::FINISHED);
        return;
    }

    transitionTo(Step::SMS_CMGS_CMD);
}

void GsmRawUartTest::handleSmsCmgsCommand()
{
    if (!commandSentThisStep)
    {
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", GSM_TEST_PHONE_NUMBER);
        sendCommand(cmd);
        return;
    }

    appendIncomingBytes();

    bool gotPrompt = responseContains(">");
    bool gotError = responseContains("ERROR");
    bool timedOut = millis() - stepStartedAt >= SMS_PROMPT_TIMEOUT_MS;

    if (!gotPrompt && !gotError && !timedOut) return;

    printStepResponse();

    if (!gotPrompt)
    {
        smsResult = SmsResult::FAIL;
        smsSkipReason = gotError ? "AT+CMGS rejected" : "no '>' prompt received";
        Serial.println("[SMS FAIL] Did not receive '>' prompt");
        transitionTo(Step::FINISHED);
        return;
    }

    Serial.println("[SMS] Sending...");
    serial.print(SMS_TEST_MESSAGE_BODY);
    serial.write(0x1A); // Ctrl+Z terminates SMS text-mode body

    transitionTo(Step::SMS_BODY);
}

void GsmRawUartTest::handleSmsBody()
{
    appendIncomingBytes();

    bool gotCmgs = responseContains("+CMGS");
    bool gotError = responseContains("ERROR");
    bool timedOut = millis() - stepStartedAt >= SMS_SEND_TIMEOUT_MS;

    if (!gotCmgs && !gotError && !timedOut) return;

    printStepResponse();

    if (gotCmgs)
    {
        smsResult = SmsResult::PASS;
        Serial.println("[SMS PASS]");
    }
    else
    {
        smsResult = SmsResult::FAIL;
        smsSkipReason = timedOut ? "timeout waiting for +CMGS" : "send error";
        Serial.println("[SMS FAIL]");
    }

    transitionTo(Step::FINISHED);
}
