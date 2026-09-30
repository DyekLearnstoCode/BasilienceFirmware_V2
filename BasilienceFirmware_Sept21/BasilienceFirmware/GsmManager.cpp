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
    lastRegistrationCheckAt = 0;

    // begin() only opens the UART and writes a few bytes, so this does not
    // block setup().
    beginSerial();
    stageStartedAt = millis();
    sendCommand("AT");
}

// Opens the GSM UART at the bench-confirmed GSM_BAUD_RATE (Config.h). This
// specific SIM800L V2 unit only ever answers at 9600 - see the GSM physical
// validation report - so there is nothing to probe for; a single fixed rate
// is both simpler and avoids repeatedly tearing down/reconfiguring the UART
// peripheral the way multi-baud cycling used to.
void GsmManager::beginSerial()
{
    serial.begin(GSM_BAUD_RATE, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    if (debugManager.shouldPrintDebug(DebugCategory::GSM))
    {
        Serial.print("[GSM] UART open at ");
        Serial.print(GSM_BAUD_RATE);
        Serial.println(" baud");
    }
}

void GsmManager::update()
{
    drainSerial();
    const unsigned long now = millis();

    // A genuine modem restart invalidates whatever state/stage this tick
    // would otherwise act on - handle it first and skip the normal handler
    // this tick if it fired.
    if (checkForModemRestart(now)) return;

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
            updateReady(now);
            break;
        case State::SENDING_SMS:
            updateSendingSms(now);
            break;
    }
}

// SIM800L emits an unsolicited "RDY" as the first line of its own boot
// sequence (RDY / +CFUN: 1 / +CPIN: READY / Call Ready / SMS Ready) - a
// reliable, distinct signature that it just powered on or reset, separate
// from any AT command/response text this class ever matches on. Ignored
// while already in WAITING_FOR_MODULE (that's exactly where a fresh boot
// belongs, and a probe there is already in flight). Seen in any other
// state, it means whatever this class currently believes about SIM/
// registration/in-flight-send state is stale.
bool GsmManager::checkForModemRestart(unsigned long now)
{
    if (state == State::WAITING_FOR_MODULE) return false;
    if (rxBuffer.indexOf("RDY") < 0) return false;

    Serial.println("[GSM] Unsolicited RDY - modem restarted, reinitializing");

    if (state == State::SENDING_SMS)
    {
        // Report the same way "cellular unavailable" is reported elsewhere
        // so NotificationManager defers the whole event rather than
        // charging this recipient a bounded TIMEOUT/ERROR retry for a
        // failure that had nothing to do with the recipient or message.
        lastResult = SendResult::MODULE_NOT_READY;
        sendStage = SendStage::NONE;
        pendingNumber = "";
        pendingMessage = "";
    }

    state = State::WAITING_FOR_MODULE;
    rxBuffer = "";
    stageStartedAt = now;
    sendCommand("AT");
    return true;
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
        Serial.println("[GSM] Module responding");
        rxBuffer = "";
        state = State::CHECKING_SIM;
        stageStartedAt = now;
        // Fire-and-forget: enables numeric/verbose +CME ERROR reporting for
        // diagnostics (see logSendError()). Not on the critical path - its
        // own "OK" is simply overwritten by the CPIN query's rxBuffer reset
        // immediately below, and CMS/CMGS behavior is unaffected either way.
        serial.print("AT+CMEE=1\r\n");
        sendCommand("AT+CPIN?");
        return;
    }

    if (now - stageStartedAt >= MODULE_PROBE_RETRY_INTERVAL_MS)
    {
        // No "OK" within this window - retry. Unbounded overall (never
        // gives up, matching this state's existing "the module may just be
        // slow to power up" policy), but each individual wait is bounded
        // and update() never blocks while doing it.
        stageStartedAt = now;
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
                lastRegistrationCheckAt = now;
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

// Registration was previously never re-checked once READY was first
// reached, so a loss of signal/registration after boot was invisible until
// a send simply hung. This reuses updateCheckingRegistration() wholesale by
// demoting back to CHECKING_REGISTRATION - the same bounded, non-blocking
// retry loop that got here the first time also handles recovering here,
// and sendSms() already rejects with NOT_REGISTERED while in that state,
// so NotificationManager defers rather than fails any send that lands in
// the brief recheck window.
void GsmManager::updateReady(unsigned long now)
{
    if (now - lastRegistrationCheckAt < REGISTRATION_HEALTH_INTERVAL_MS) return;

    state = State::CHECKING_REGISTRATION;
    stageStartedAt = now;
    rxBuffer = "";
    sendCommand("AT+CREG?");
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
            if (rxBuffer.indexOf("+CMS ERROR") >= 0 || rxBuffer.indexOf("+CME ERROR") >= 0 ||
                rxBuffer.indexOf("ERROR") >= 0)
            {
                logSendError(rxBuffer);
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

// Best-effort diagnostic only - never changes SendResult or control flow.
// Extracts and logs the numeric code from "+CMS ERROR: <n>" or
// "+CME ERROR: <n>" (present because AT+CMEE=1 was sent once at module
// detection) so a real failure reason survives in the serial log instead of
// a bare "ERROR". Contains no phone number, so nothing here needs masking.
void GsmManager::logSendError(const String& response) const
{
    int tagIdx = response.indexOf("+CMS ERROR:");
    size_t tagLen = 11; // strlen("+CMS ERROR:")
    if (tagIdx < 0)
    {
        tagIdx = response.indexOf("+CME ERROR:");
        tagLen = 11; // strlen("+CME ERROR:")
    }

    if (tagIdx < 0)
    {
        Serial.println("[GSM] SMS send ERROR");
        return;
    }

    int end = response.indexOf('\r', tagIdx);
    String code = (end >= 0) ? response.substring(tagIdx + tagLen, end)
                              : response.substring(tagIdx + tagLen);
    code.trim();

    Serial.print("[GSM] SMS send failed: ");
    Serial.println(code);
}

String GsmManager::maskPhoneNumber(const String& phoneNumber)
{
    // Canonical form is always 13 chars: keep "+63" + first subscriber digit
    // + next 2 digits visible, mask the middle 4, keep the last 3.
    // e.g. +639171234567 -> +63917****567
    if (phoneNumber.length() != 13) return "****";
    return phoneNumber.substring(0, 6) + "****" + phoneNumber.substring(10);
}
