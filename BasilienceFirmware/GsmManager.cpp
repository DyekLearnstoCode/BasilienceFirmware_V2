#include "GsmManager.h"
#include "Config.h"
#include "Globals.h"

void GsmManager::begin()
{
    Serial.println("[GSM] Initializing SIM800L");

    state = State::WAITING_FOR_MODULE;
    lastResult = SendResult::NONE;
    sendStage = SendStage::NONE;
    rxBuffer = "";
    pendingNumber = "";
    pendingMessage = "";
    baudCandidateIndex = 0;

    // First probe fires immediately; begin() only writes a few bytes to the
    // UART and returns, so this does not block setup().
    beginSerialAtCurrentBaudCandidate();
    stageStartedAt = millis();
    sendCommand("AT");
}

// (Re)opens the GSM UART at BAUD_CANDIDATES[baudCandidateIndex]. HardwareSerial::
// begin() on this core tears down and reconfigures the peripheral itself, so
// calling it again with a different rate - which updateWaitingForModule() does
// each time a candidate's window expires with no "OK" - is safe and does not
// require an explicit end() first.
void GsmManager::beginSerialAtCurrentBaudCandidate()
{
    const unsigned long baud = BAUD_CANDIDATES[baudCandidateIndex];
    serial.begin(baud, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    if (debugManager.shouldPrintDebug(DebugCategory::GSM))
    {
        Serial.print("[GSM] Probing at ");
        Serial.print(baud);
        Serial.println(" baud");
    }
}

void GsmManager::update()
{
    drainSerial();
    const unsigned long now = millis();

    switch (state)
    {
        case State::WAITING_FOR_MODULE:
            updateWaitingForModule(now);
            break;
        case State::CHECKING_SIM:
            updateCheckingSim(now);
            break;
        case State::CHECKING_REGISTRATION:
            updateCheckingRegistration(now);
            break;
        case State::READY:
            // Idle. sendSms() drives the next transition.
            break;
        case State::SENDING_SMS:
            updateSendingSms(now);
            break;
    }
}

bool GsmManager::sendSms(const String& phoneNumber, const String& message)
{
    if (state == State::SENDING_SMS)
    {
        lastResult = SendResult::BUSY;
        return false;
    }
    if (state == State::WAITING_FOR_MODULE)
    {
        lastResult = SendResult::MODULE_NOT_READY;
        return false;
    }
    if (state == State::CHECKING_SIM)
    {
        lastResult = SendResult::SIM_NOT_READY;
        return false;
    }
    if (state == State::CHECKING_REGISTRATION)
    {
        lastResult = SendResult::NOT_REGISTERED;
        return false;
    }
    if (!isValidCanonicalPhilippineMobile(phoneNumber))
    {
        lastResult = SendResult::INVALID_NUMBER;
        return false;
    }

    pendingNumber = phoneNumber;
    pendingMessage = message;
    lastResult = SendResult::NONE;
    state = State::SENDING_SMS;
    beginSendStage(SendStage::SET_TEXT_MODE);
    return true;
}

bool GsmManager::isBusy() const
{
    return state == State::SENDING_SMS;
}

bool GsmManager::isReady() const
{
    return state == State::READY;
}

GsmManager::State GsmManager::getState() const
{
    return state;
}

GsmManager::SendResult GsmManager::getLastResult() const
{
    return lastResult;
}

bool GsmManager::isValidCanonicalPhilippineMobile(const String& phoneNumber)
{
    // Structural-only: '+', "63", then exactly 10 digits starting with '9'.
    // Deliberately no carrier-prefix table.
    if (phoneNumber.length() != 13) return false;
    if (phoneNumber.charAt(0) != '+') return false;
    if (phoneNumber.charAt(1) != '6' || phoneNumber.charAt(2) != '3') return false;
    if (phoneNumber.charAt(3) != '9') return false;

    for (unsigned int i = 3; i < 13; i++)
    {
        if (!isDigit(phoneNumber.charAt(i))) return false;
    }
    return true;
}

void GsmManager::drainSerial()
{
    while (serial.available() > 0)
    {
        if (rxBuffer.length() >= RX_BUFFER_CAP)
        {
            // No expected token has matched in RX_BUFFER_CAP bytes of
            // response - drop the stale prefix rather than growing forever
            // or wedging permanently on noise/garbage.
            rxBuffer = "";
        }
        rxBuffer += static_cast<char>(serial.read());
    }
}

void GsmManager::sendCommand(const char* command)
{
    rxBuffer = "";
    serial.print(command);
    serial.print("\r\n");
}

void GsmManager::updateWaitingForModule(unsigned long now)
{
    if (rxBuffer.indexOf("OK") >= 0)
    {
        Serial.print("[GSM] Module responding at ");
        Serial.print(BAUD_CANDIDATES[baudCandidateIndex]);
        Serial.println(" baud");
        rxBuffer = "";
        state = State::CHECKING_SIM;
        stageStartedAt = now;
        sendCommand("AT+CPIN?");
        return;
    }

    if (now - stageStartedAt >= MODULE_PROBE_RETRY_INTERVAL_MS)
    {
        stageStartedAt = now;

        // No "OK" within this candidate's window - move to the next baud
        // and reopen the UART there before probing again. Wraps around
        // indefinitely (same "never give up, just keep polling" policy this
        // state already used for a single baud) so a module that's slow to
        // power up is still found eventually, not just on this pass.
        baudCandidateIndex = (baudCandidateIndex + 1) % BAUD_CANDIDATE_COUNT;
        beginSerialAtCurrentBaudCandidate();
        sendCommand("AT");
    }
}

void GsmManager::updateCheckingSim(unsigned long now)
{
    if (rxBuffer.indexOf("+CPIN: READY") >= 0)
    {
        Serial.println("[GSM] SIM ready");
        rxBuffer = "";
        state = State::CHECKING_REGISTRATION;
        stageStartedAt = now;
        sendCommand("AT+CREG?");
        return;
    }

    // Covers both an explicit error/locked-SIM response and no response at
    // all within the window - either way, retry the same query rather than
    // waiting indefinitely.
    if (now - stageStartedAt >= SIM_CHECK_RETRY_INTERVAL_MS)
    {
        stageStartedAt = now;
        sendCommand("AT+CPIN?");
    }
}

void GsmManager::updateCheckingRegistration(unsigned long now)
{
    // SIM800L is 2G/GPRS-only and has no EPS stack, so registration is
    // checked with legacy circuit-switched AT+CREG? rather than AT+CEREG?
    // (the previous module was LTE Cat1 and needed EPS registration).
    // Response shape: "+CREG: <n>,<stat>[,...]" - stat 1 = registered home,
    // 5 = registered roaming. Same shape and status codes as CEREG, so only
    // the command/tag string changes here - the status digit still always
    // immediately follows the first comma in both the 2-value and extended
    // (5-value) forms.
    int tagIdx = rxBuffer.indexOf("+CREG:");
    if (tagIdx >= 0)
    {
        int commaIdx = rxBuffer.indexOf(',', tagIdx);
        if (commaIdx >= 0 && commaIdx + 1 < (int)rxBuffer.length())
        {
            char stat = rxBuffer.charAt(commaIdx + 1);
            if (stat == '1' || stat == '5')
            {
                Serial.println("[GSM] Registered");
                rxBuffer = "";
                state = State::READY;
                stageStartedAt = now;
                return;
            }
        }
    }

    if (now - stageStartedAt >= REGISTRATION_RETRY_INTERVAL_MS)
    {
        stageStartedAt = now;
        sendCommand("AT+CREG?");
    }
}

void GsmManager::beginSendStage(SendStage stage)
{
    sendStage = stage;
    stageStartedAt = millis();
    rxBuffer = "";

    if (stage == SendStage::SET_TEXT_MODE)
    {
        sendCommand("AT+CMGF=1");
    }
    else if (stage == SendStage::AWAIT_PROMPT)
    {
        serial.print("AT+CMGS=\"");
        serial.print(pendingNumber);
        serial.print("\"\r\n");
    }
    // AWAIT_SEND_RESULT is entered only after the message body + Ctrl+Z have
    // already been written by updateSendingSms(); nothing to send here.
}

void GsmManager::updateSendingSms(unsigned long now)
{
    switch (sendStage)
    {
        case SendStage::SET_TEXT_MODE:
            if (rxBuffer.indexOf("OK") >= 0)
            {
                beginSendStage(SendStage::AWAIT_PROMPT);
                return;
            }
            if (now - stageStartedAt >= TEXT_MODE_TIMEOUT_MS)
            {
                finishSend(SendResult::ERROR);
            }
            break;

        case SendStage::AWAIT_PROMPT:
            if (rxBuffer.indexOf('>') >= 0)
            {
                serial.print(pendingMessage);
                serial.write(0x1A); // Ctrl+Z submits the SMS body
                sendStage = SendStage::AWAIT_SEND_RESULT;
                stageStartedAt = now;
                rxBuffer = "";
                return;
            }
            if (now - stageStartedAt >= PROMPT_TIMEOUT_MS)
            {
                finishSend(SendResult::TIMEOUT);
            }
            break;

        case SendStage::AWAIT_SEND_RESULT:
            if (rxBuffer.indexOf("+CMGS:") >= 0 && rxBuffer.indexOf("OK") >= 0)
            {
                finishSend(SendResult::SUCCESS);
                return;
            }
            if (rxBuffer.indexOf("+CMS ERROR") >= 0 || rxBuffer.indexOf("ERROR") >= 0)
            {
                finishSend(SendResult::ERROR);
                return;
            }
            if (now - stageStartedAt >= SEND_RESULT_TIMEOUT_MS)
            {
                finishSend(SendResult::TIMEOUT);
            }
            break;

        case SendStage::NONE:
            break;
    }
}

void GsmManager::finishSend(SendResult result)
{
    lastResult = result;
    sendStage = SendStage::NONE;
    state = State::READY;

    if (result == SendResult::SUCCESS)
    {
        Serial.print("[GSM] SMS sent to ");
        Serial.println(maskPhoneNumber(pendingNumber));
    }
    else
    {
        const char* reason = "ERROR";
        if (result == SendResult::TIMEOUT) reason = "TIMEOUT";
        Serial.print("[GSM] SMS failed: ");
        Serial.println(reason);
    }

    pendingNumber = "";
    pendingMessage = "";
}

String GsmManager::maskPhoneNumber(const String& phoneNumber)
{
    // Canonical form is always 13 chars: keep "+63" + first subscriber digit
    // + next 2 digits visible, mask the middle 4, keep the last 3.
    // e.g. +639171234567 -> +63917****567
    if (phoneNumber.length() != 13) return "****";
    return phoneNumber.substring(0, 6) + "****" + phoneNumber.substring(10);
}
