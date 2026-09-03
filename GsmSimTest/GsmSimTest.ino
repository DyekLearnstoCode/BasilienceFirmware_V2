// Standalone SIM800L test sketch - answers one question: is the SIM card
// actually being read by the module. Not part of the main Basilience
// firmware build (separate sketch folder, so it won't get pulled into
// BasilienceFirmware.ino's compile) - flash this by itself onto the ESP32,
// watch the Serial Monitor at 115200 baud, and read the printed results.
//
// Wiring (see GSM_RX_PIN/GSM_TX_PIN below - currently 16/17, swapped from
// the main firmware's Config.h pins of 36/23 to rule out a pin-specific
// fault after a first run showed the module healthy but every SIM command
// erroring):
//   ESP32 GPIO 16 (RX) <- SIM800L TXD
//   ESP32 GPIO 17 (TX) -> SIM800L RXD
//   Common ground between ESP32 and the SIM800L's own power supply.
//   SIM800L needs its own 4V-ish, multi-amp-capable supply - the ESP32's
//   3.3V/5V rail cannot drive it, especially not during the transmit current
//   spikes. If the module won't even answer "AT", check power first.
//
// What "SIM is being read" actually means here, in order of what each
// command proves:
//   AT          - the module itself is alive and talking over UART.
//   AT+CPIN?    - a SIM is physically inserted and not PIN-locked.
//   AT+CCID     - reads the SIM's own ICCID (its serial number) straight off
//                 the card. This is the clearest possible proof the SIM is
//                 being read, not just detected as "present."
//   AT+CIMI     - reads the SIM's IMSI (subscriber identity), a second,
//                 independent read from the card itself.
//   AT+CSQ      - signal quality, useful context but not SIM-specific.
//   AT+CREG?    - network registration status, useful context but not
//                 SIM-specific (a SIM can read fine and still fail to
//                 register, e.g. no signal, wrong APN, expired load).

#include <Arduino.h>

// Swapped from the main firmware's GSM_RX_PIN=36/GSM_TX_PIN=23 to rule out a
// pin-specific fault, now that a first run showed the module itself healthy
// (clean AT/CSQ/CREG replies) but every SIM-specific command erroring. 16/17
// are plain GPIOs (not input-only like 36, not strapping pins), and nothing
// else is running in this standalone sketch to conflict with them.
//   ESP32 GPIO 16 (RX) <- SIM800L TXD
//   ESP32 GPIO 17 (TX) -> SIM800L RXD
// Rewire accordingly before flashing this version.
static const uint8_t GSM_RX_PIN = 16;
static const uint8_t GSM_TX_PIN = 17;

static const unsigned long BAUD_CANDIDATES[] = {115200UL, 9600UL, 57600UL, 38400UL};
static const uint8_t BAUD_CANDIDATE_COUNT =
    sizeof(BAUD_CANDIDATES) / sizeof(BAUD_CANDIDATES[0]);

HardwareSerial gsm(1);

// Sends `command`, waits up to `timeoutMs` for any response, and returns the
// raw bytes received. Blocking is fine here - this is a one-shot diagnostic
// tool, not the always-on cultivation firmware, so there is nothing else
// that needs to keep running underneath it.
String sendATAndWait(const char* command, unsigned long timeoutMs)
{
    while (gsm.available()) gsm.read(); // drop any stale bytes first

    gsm.print(command);
    gsm.print("\r\n");

    String response;
    unsigned long startedAt = millis();
    while (millis() - startedAt < timeoutMs)
    {
        while (gsm.available())
        {
            response += (char)gsm.read();
            startedAt = millis(); // keep waiting while bytes are still arriving
        }
    }
    return response;
}

void printResult(const char* label, const String& response)
{
    Serial.print("[");
    Serial.print(label);
    Serial.println("]");
    if (response.length() == 0)
    {
        Serial.println("  (no response - module did not answer in time)");
    }
    else
    {
        Serial.println("  " + response);
    }
}

// Cycles through candidate bauds repeatedly - not just once - until the
// module answers "AT" with "OK" or PROBE_BUDGET_MS runs out. A single pass
// (roughly 2s x 4 bauds = 8s) is too impatient: SIM800L modules commonly
// take several seconds after power-on before they'll answer anything, and
// the production GsmManager this mirrors never gives up at all, it keeps
// cycling indefinitely. Returns true and leaves `gsm` open at the working
// baud if found.
bool findModuleBaud()
{
    const unsigned long PROBE_BUDGET_MS = 60000UL; // total time before giving up
    unsigned long startedAt = millis();
    uint8_t pass = 1;

    while (millis() - startedAt < PROBE_BUDGET_MS)
    {
        Serial.print("--- Pass ");
        Serial.print(pass++);
        Serial.println(" ---");

        for (uint8_t i = 0; i < BAUD_CANDIDATE_COUNT; i++)
        {
            unsigned long baud = BAUD_CANDIDATES[i];
            Serial.print("Probing at ");
            Serial.print(baud);
            Serial.println(" baud...");

            gsm.begin(baud, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
            delay(200); // let the UART settle after (re)configuring it

            // Two tries per baud, not one - the module can eat the very
            // first byte it receives while still finishing its own boot.
            for (uint8_t attempt = 0; attempt < 2; attempt++)
            {
                String response = sendATAndWait("AT", 2000);
                if (response.indexOf("OK") >= 0)
                {
                    Serial.print("Module responding at ");
                    Serial.print(baud);
                    Serial.println(" baud.");
                    return true;
                }
            }
        }
    }
    return false;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("=== SIM800L SIM Read Test ===");
    Serial.println();

    if (!findModuleBaud())
    {
        Serial.println();
        Serial.println("Module never answered \"AT\" on any candidate baud.");
        Serial.println("Check: power to the SIM800L (needs its own supply, not the");
        Serial.println("ESP32 rail), the RX/TX wiring (they cross: ESP32 RX to");
        Serial.println("module TXD, ESP32 TX to module RXD), and a shared ground.");
        return;
    }

    Serial.println();
    printResult("AT+CPIN? (SIM present / unlocked)", sendATAndWait("AT+CPIN?", 3000));

    Serial.println();
    printResult("AT+CCID (SIM serial number, read from the card)", sendATAndWait("AT+CCID", 3000));

    Serial.println();
    printResult("AT+CIMI (subscriber identity, read from the card)", sendATAndWait("AT+CIMI", 3000));

    Serial.println();
    printResult("AT+CSQ (signal quality)", sendATAndWait("AT+CSQ", 3000));

    Serial.println();
    printResult("AT+CREG? (network registration)", sendATAndWait("AT+CREG?", 3000));

    Serial.println();
    Serial.println("=== Done ===");
    Serial.println("If AT+CCID and AT+CIMI both returned real numbers (not");
    Serial.println("\"ERROR\" or blank), the SIM is being read correctly.");
    Serial.println("If AT+CPIN? did not say \"+CPIN: READY\", the SIM either");
    Serial.println("isn't seated properly or is PIN-locked.");
}

void loop()
{
    // One-shot test - nothing to repeat. Re-run setup() by resetting the
    // board if you want to test again (e.g. after reseating the SIM).
}
