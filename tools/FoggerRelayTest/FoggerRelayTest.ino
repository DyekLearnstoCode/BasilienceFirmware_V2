// Standalone bench-test sketch for the Basilience fogger relay.
// Uses the same GPIO as production (ActuatorManager::FOGGER_PIN, see
// Config.h): GPIO26. Polarity (active-high vs active-low) is switchable
// live over serial, since cheap relay boards vary (some have an H/L
// trigger-mode jumper, some don't) and it's faster to test both from one
// flash than to reflash for each guess.
//
// Usage:
//   1. Flash this sketch alone (not the full firmware) and open the Serial
//      Monitor at 115200 baud, line ending "Newline".
//   2. Wire the relay module's signal pin to GPIO26, plus 5V/GND, with the
//      fogger's own power on the relay's switched side (never through the
//      ESP32 pin directly).
//   3. Send "1" to turn the fogger relay ON, "0" to turn it OFF.
//   4. Send "p" to run a single pulse test (ON for FOGGER_PULSE_MS, then
//      auto-OFF) - this mirrors a normal automatic fogging burst.
//   5. Send "t" to toggle repeatedly a few times as a quick relay-click test.
//   6. If nothing clicks: send "h" to switch to active-high mode, or "l" to
//      switch to active-low mode (starts active-low), then retry "1"/"t".
//      Whichever polarity actually clicks the relay is the one to keep.
//
// Safety: a relay left ON is auto-cut after FOGGER_SAFETY_TIMEOUT_MS even if
// no "0" is sent, mirroring the OPERATION_TIMEOUT_MS hard cap in the real
// firmware (ActuatorManager.cpp) so a forgotten test can't leave the fogger
// running unattended.

#include <Arduino.h>

constexpr uint8_t FOGGER_PIN = 26;
constexpr unsigned long FOGGER_PULSE_MS = 3000UL;
constexpr unsigned long FOGGER_SAFETY_TIMEOUT_MS = 300000UL; // 5 minutes, matches OPERATION_TIMEOUT_MS

bool foggerOn = false;
bool activeLow = true; // "l" / "h" flip this live to test both relay polarities
unsigned long turnedOnAt = 0;

void applyFoggerPin()
{
    digitalWrite(FOGGER_PIN, foggerOn == activeLow ? LOW : HIGH);
}

void setFogger(bool on)
{
    foggerOn = on;
    applyFoggerPin();
    turnedOnAt = on ? millis() : 0;

    Serial.print(F("Fogger relay -> "));
    Serial.println(on ? F("ON") : F("OFF"));
}

void setPolarity(bool newActiveLow)
{
    activeLow = newActiveLow;
    applyFoggerPin(); // re-drive the pin for the current on/off state under the new polarity

    Serial.print(F("Polarity -> "));
    Serial.println(activeLow ? F("ACTIVE-LOW (IN=LOW energizes)") : F("ACTIVE-HIGH (IN=HIGH energizes)"));
}

void printMenu()
{
    Serial.println();
    Serial.println(F("=== Basilience Fogger Relay Test (GPIO26) ==="));
    Serial.println(F("1 = relay ON   0 = relay OFF   p = pulse test   t = toggle test"));
    Serial.println(F("h = active-high mode   l = active-low mode"));
    Serial.println();
}

void pulseTest()
{
    Serial.print(F("Pulsing fogger for "));
    Serial.print(FOGGER_PULSE_MS);
    Serial.println(F(" ms..."));

    setFogger(true);
    delay(FOGGER_PULSE_MS);
    setFogger(false);

    Serial.println(F("Pulse complete."));
}

void toggleTest()
{
    Serial.println(F("Toggling relay 6 times (500ms each) - listen/watch for clean clicks..."));

    for (int i = 0; i < 6; i++)
    {
        setFogger(!foggerOn);
        delay(500);
    }

    setFogger(false);
    Serial.println(F("Toggle test complete."));
}

void handleCommand(const String &cmd)
{
    if (cmd == "1")
    {
        setFogger(true);
    }
    else if (cmd == "0")
    {
        setFogger(false);
    }
    else if (cmd == "p")
    {
        pulseTest();
    }
    else if (cmd == "t")
    {
        toggleTest();
    }
    else if (cmd == "h")
    {
        setPolarity(false);
    }
    else if (cmd == "l")
    {
        setPolarity(true);
    }
    else
    {
        printMenu();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(FOGGER_PIN, OUTPUT);
    applyFoggerPin(); // foggerOn=false, activeLow=true at boot -> pin HIGH (off)

    printMenu();
}

void loop()
{
    if (foggerOn && millis() - turnedOnAt >= FOGGER_SAFETY_TIMEOUT_MS)
    {
        Serial.println(F("[SAFETY] Fogger safety timeout reached - forcing OFF."));
        setFogger(false);
    }

    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0)
        {
            handleCommand(cmd);
        }
    }
}
