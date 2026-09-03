/*
  FanPwmSpeedTest
  ---------------
  Standalone bench test for the Canopy Fan and the Blower, the two
  MOSFET-driven DC fans in BasilienceFirmware. This is a separate sketch,
  not part of the automation firmware. Flash this alone, on the bench, to
  find out two things about the real hardware:
    1. The lowest PWM percent that still spins each fan reliably. Below
       that percent a MOSFET-driven DC fan usually just buzzes or twitches
       instead of turning, since there is not enough average power to
       overcome its own starting friction.
    2. Whether the fan speed actually changes in a way you can see or hear
       across the 0-100% range, or whether large parts of that range feel
       the same to you.

  Wiring (matches production, see Config.h and ActuatorManager.cpp). Both
  fans are plain two-wire DC fans, power and ground only, no built-in
  tachometer or PWM-input line, so the MOSFET is doing all of the speed
  control from outside the fan itself:
    ESP32 GPIO 17 (CANOPY_FAN_PIN) -> gate of the canopy fan's MOSFET
    ESP32 GPIO 27 (BLOWER_PIN)     -> gate of the blower's MOSFET
    MOSFET source -> GND, shared with the fan's own power supply GND
    MOSFET drain  -> fan negative lead
    Fan positive lead -> the fan's own DC supply, not the ESP32 5V/3V3 pin
    A flyback diode across the fan terminals, unless the driver board you
    are using already has one built in
  If you are testing one fan at a time on the bench, only wire that one.
  Because there is no tachometer wire, this sketch has no way to read RPM
  back, so judging each step by ear and eye during the sweep is the only
  way to find the usable range, there is nothing to log instead of that.

  The PWM frequency and resolution below (5000 Hz, 8-bit) are copied from
  Config.h's CANOPY_BLOWER_PWM_FREQUENCY_HZ and
  CANOPY_BLOWER_PWM_RESOLUTION_BITS, and percentToDuty() below is copied
  from ActuatorManager.cpp's own conversion. This is deliberate: a percent
  you find working here will behave the same once it is set through the
  real app and firmware, because both sides are driving the MOSFET the
  same way.

  Use the Serial Monitor at 115200 baud, line ending set to Newline.

  Commands:
    c<percent>   set the Canopy Fan, for example c50
    b<percent>   set the Blower, for example b30
    a<percent>   set both to the same percent, for example a75
    c0 / b0 / a0 turn a fan off
    sc           sweep the Canopy Fan from 0 to 100 percent in 5 percent
                 steps, holding each step for 4 seconds
    sb           sweep the Blower the same way
    sa           sweep both fans together the same way
    x            stop a running sweep early
    ?            print this menu again

  While a sweep runs, watch and listen to the fan at each step and write
  down the lowest percent where it spins smoothly and continuously rather
  than just buzzing.
*/

#include <Arduino.h>

constexpr uint8_t CANOPY_FAN_PIN = 17;
constexpr uint8_t BLOWER_PIN = 27;
constexpr uint32_t PWM_FREQUENCY_HZ = 5000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint16_t PWM_MAX_DUTY = (1u << PWM_RESOLUTION_BITS) - 1;

constexpr uint8_t SWEEP_STEP_PERCENT = 5;
constexpr unsigned long SWEEP_STEP_DWELL_MS = 4000UL;

bool sweepAbort = false;

uint8_t percentToDuty(uint8_t percent)
{
    if (percent > 100) percent = 100;
    return (uint8_t)((percent / 100.0f) * PWM_MAX_DUTY);
}

void setFan(uint8_t pin, const char* label, uint8_t percent)
{
    uint8_t duty = percentToDuty(percent);
    ledcWrite(pin, duty);
    Serial.print("[");
    Serial.print(label);
    Serial.print("] set to ");
    Serial.print(percent);
    Serial.print("% (duty ");
    Serial.print(duty);
    Serial.print("/");
    Serial.print(PWM_MAX_DUTY);
    Serial.println(")");
}

void printMenu()
{
    Serial.println();
    Serial.println("=== Fan PWM Speed Test ===");
    Serial.println("c<percent>  set Canopy Fan, e.g. c50");
    Serial.println("b<percent>  set Blower, e.g. b30");
    Serial.println("a<percent>  set both, e.g. a75");
    Serial.println("sc / sb / sa  sweep canopy / blower / both, 0-100% in 5% steps, 4s per step");
    Serial.println("x           stop a running sweep");
    Serial.println("?           show this menu");
    Serial.println("Watch and listen at each step. Note the lowest percent where the fan spins");
    Serial.println("smoothly and continuously, not just buzzing or twitching.");
    Serial.println();
}

void runSweep(bool doCanopy, bool doBlower)
{
    sweepAbort = false;
    Serial.println("--- Sweep starting, send x to stop early ---");
    for (int percent = 0; percent <= 100; percent += SWEEP_STEP_PERCENT)
    {
        if (doCanopy) setFan(CANOPY_FAN_PIN, "CANOPY", (uint8_t)percent);
        if (doBlower) setFan(BLOWER_PIN, "BLOWER", (uint8_t)percent);

        unsigned long stepStart = millis();
        while (millis() - stepStart < SWEEP_STEP_DWELL_MS)
        {
            if (Serial.available() && Serial.read() == 'x')
            {
                sweepAbort = true;
                break;
            }
        }
        if (sweepAbort) break;
    }
    if (doCanopy) setFan(CANOPY_FAN_PIN, "CANOPY", 0);
    if (doBlower) setFan(BLOWER_PIN, "BLOWER", 0);
    Serial.println(sweepAbort ? "--- Sweep stopped early ---" : "--- Sweep complete ---");
}

void handleCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "?") { printMenu(); return; }
    if (cmd == "x") { sweepAbort = true; return; }
    if (cmd == "sc") { runSweep(true, false); return; }
    if (cmd == "sb") { runSweep(false, true); return; }
    if (cmd == "sa") { runSweep(true, true); return; }

    char target = cmd.charAt(0);
    if (target != 'c' && target != 'b' && target != 'a')
    {
        Serial.println("Unrecognized command, send ? for the menu.");
        return;
    }
    String numPart = cmd.substring(1);
    if (numPart.length() == 0 || !isDigit(numPart.charAt(0)))
    {
        Serial.println("Missing percent, e.g. c50");
        return;
    }
    int percent = numPart.toInt();
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    if (target == 'c' || target == 'a') setFan(CANOPY_FAN_PIN, "CANOPY", (uint8_t)percent);
    if (target == 'b' || target == 'a') setFan(BLOWER_PIN, "BLOWER", (uint8_t)percent);
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    ledcAttach(CANOPY_FAN_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(BLOWER_PIN, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcWrite(CANOPY_FAN_PIN, 0);
    ledcWrite(BLOWER_PIN, 0);

    printMenu();
}

void loop()
{
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }
}
