/*
  Pcf8574PhPumpTest
  ------------------
  Standalone bench test for a PCF8574 I2C I/O expander driving the pH Up
  and pH Down pump relays. This is a separate sketch, not part of the
  automation firmware, and does not use BasilienceFirmware's own direct-
  GPIO pH pump wiring (Config.h's PH_UP_PUMP_PIN / PH_DOWN_PUMP_PIN, driven
  straight from the ESP32). Use this sketch on the bench to confirm the
  expander board itself, its I2C address, and the two relay channels work
  before wiring pumps to it for real.

  Why a raw I2C write and no PCF8574 library: the PCF8574 protocol is one
  instruction. Writing a single byte to its I2C address sets all 8 output
  pins at once (there is no per-pin write), and reading a byte back reads
  all 8 pins at once. That is simple enough to do directly with Wire, so
  this sketch has no extra library dependency to install.

  Wiring:
    ESP32 GPIO 21 (SDA) -> PCF8574 SDA
    ESP32 GPIO 22 (SCL) -> PCF8574 SCL
      These are the same two pins Config.h already assigns to the RTC
      (RTC_SDA_PIN / RTC_SCL_PIN), since I2C is a shared bus and any
      future PCF8574 would sit on it alongside the RTC.
    PCF8574 VCC -> 5V (match this to whatever logic level your relay
      board's IN pins expect, commonly 5V)
    PCF8574 GND -> common GND with the ESP32 and the relay board
    PCF8574 address pins A0, A1, A2 -> GND for I2C address 0x20 (the
      default this sketch uses). If you are using a PCF8574A instead of
      a PCF8574, its base address is 0x38, not 0x20, change
      PCF8574_I2C_ADDRESS below to match.
    Most PCF8574 breakout boards already carry their own SDA/SCL pull-up
    resistors. If you are wiring a bare PCF8574 chip instead of a
    breakout board, add a 4.7k-10k resistor from SDA to VCC and another
    from SCL to VCC.
    PCF8574 P0 -> relay module IN1 (pH Up pump relay)
    PCF8574 P1 -> relay module IN2 (pH Down pump relay)
    Relay module GND -> common GND
    Relay module VCC -> its own logic/coil supply per that board's rating
    Relay COM/NO wired in series with the pH Up / pH Down pump's power
    line, the same place PH_UP_PUMP_PIN / PH_DOWN_PUMP_PIN switch it
    directly in production, just switched through the expander and relay
    here instead.

  A PCF8574 output pin is open-drain ("quasi-bidirectional"): writing 1
  only weakly pulls the pin high, it cannot source real current, while
  writing 0 actively sinks current to GND. This is exactly what a relay
  board's opto-isolated IN pin needs (it wants to be pulled to GND to
  trigger), so it is a good match, but do not expect a PCF8574 output to
  drive anything that needs to be actively sourced rather than sunk.

  Relay polarity: most common relay boards are active LOW, meaning
  pulling IN LOW turns the relay ON and leaving it HIGH (or floating with
  the board's own pull-up) keeps it OFF. This sketch assumes that and
  defaults to RELAY_ACTIVE_LOW true below. A PCF8574 also powers up with
  all 8 pins HIGH before this sketch ever runs, so with an active LOW
  relay board both relays are guaranteed OFF at power-on. If your relay
  board is active HIGH instead, both relays would be ON at power-on
  before this sketch can correct that. Confirm your board's polarity
  before wiring it to a real pump, and set RELAY_ACTIVE_LOW to false only
  if you have also re-wired so the power-on state is safe.

  As a safety net for testing real dosing pumps, each relay auto-shuts-off
  after PUMP_MAX_ON_MS (10 seconds here, longer than production's 5 second
  PH_DOSING_TIME so you have room to observe it, but still bounded) so a
  forgotten "on" command cannot dose indefinitely. Turning pH Up and pH
  Down on at the same time is refused, mirroring the mutual exclusion
  ActuatorManager.cpp already enforces in production between PH_UP_PUMP
  and PH_DOWN_PUMP.

  Use the Serial Monitor at 115200 baud, line ending set to Newline.

  Commands:
    u1   turn the pH Up relay on
    u0   turn the pH Up relay off
    d1   turn the pH Down relay on
    d0   turn the pH Down relay off
    x    turn both relays off immediately
    s    print the current relay status and the raw PCF8574 byte
    ?    print this menu again
*/

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t PCF8574_I2C_ADDRESS = 0x20;
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;

constexpr uint8_t PH_UP_BIT = 0;   // PCF8574 P0
constexpr uint8_t PH_DOWN_BIT = 1; // PCF8574 P1

constexpr bool RELAY_ACTIVE_LOW = true;
constexpr unsigned long PUMP_MAX_ON_MS = 10000UL;

uint8_t portState = 0xFF; // all pins released HIGH, matches PCF8574 power-on state
bool phUpOn = false;
bool phDownOn = false;
unsigned long phUpOnAt = 0;
unsigned long phDownOnAt = 0;

bool writePort(uint8_t value)
{
    Wire.beginTransmission(PCF8574_I2C_ADDRESS);
    Wire.write(value);
    uint8_t result = Wire.endTransmission();
    if (result != 0)
    {
        Serial.print("I2C write failed, error code ");
        Serial.println(result);
        return false;
    }
    portState = value;
    return true;
}

void setBit(uint8_t bit, bool energize)
{
    // Active LOW board: energizing writes 0, de-energizing writes 1.
    // Active HIGH board: the opposite.
    bool pinHigh = RELAY_ACTIVE_LOW ? !energize : energize;

    uint8_t next = portState;
    if (pinHigh) next |= (1 << bit);
    else next &= ~(1 << bit);
    writePort(next);
}

void printStatus()
{
    Serial.print("[STATUS] pH Up = ");
    Serial.print(phUpOn ? "ON" : "off");
    Serial.print(", pH Down = ");
    Serial.print(phDownOn ? "ON" : "off");
    Serial.print(", raw port = 0b");
    for (int i = 7; i >= 0; i--) Serial.print((portState >> i) & 1);
    Serial.println();
}

void printMenu()
{
    Serial.println();
    Serial.println("=== PCF8574 pH Pump Relay Test ===");
    Serial.println("u1 / u0   pH Up relay on / off");
    Serial.println("d1 / d0   pH Down relay on / off");
    Serial.println("x         both relays off now");
    Serial.println("s         print status");
    Serial.println("?         show this menu");
    Serial.print("Each relay auto-shuts-off after ");
    Serial.print(PUMP_MAX_ON_MS / 1000UL);
    Serial.println(" seconds. pH Up and pH Down cannot both be on at once.");
    Serial.println();
}

void turnOffPhUp()
{
    if (!phUpOn) return;
    setBit(PH_UP_BIT, false);
    phUpOn = false;
    Serial.println("[PH UP] off");
}

void turnOffPhDown()
{
    if (!phDownOn) return;
    setBit(PH_DOWN_BIT, false);
    phDownOn = false;
    Serial.println("[PH DOWN] off");
}

void turnOnPhUp()
{
    if (phDownOn)
    {
        Serial.println("Refused: pH Down is already on, turn it off first.");
        return;
    }
    setBit(PH_UP_BIT, true);
    phUpOn = true;
    phUpOnAt = millis();
    Serial.println("[PH UP] on");
}

void turnOnPhDown()
{
    if (phUpOn)
    {
        Serial.println("Refused: pH Up is already on, turn it off first.");
        return;
    }
    setBit(PH_DOWN_BIT, true);
    phDownOn = true;
    phDownOnAt = millis();
    Serial.println("[PH DOWN] on");
}

void handleCommand(String cmd)
{
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "?") { printMenu(); return; }
    if (cmd == "s") { printStatus(); return; }
    if (cmd == "x") { turnOffPhUp(); turnOffPhDown(); return; }
    if (cmd == "u1") { turnOnPhUp(); return; }
    if (cmd == "u0") { turnOffPhUp(); return; }
    if (cmd == "d1") { turnOnPhDown(); return; }
    if (cmd == "d0") { turnOffPhDown(); return; }

    Serial.println("Unrecognized command, send ? for the menu.");
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Wire.begin(SDA_PIN, SCL_PIN);
    writePort(0xFF); // both relays de-energized before anything else runs

    Serial.print("Probing PCF8574 at address 0x");
    Serial.println(PCF8574_I2C_ADDRESS, HEX);
    Wire.beginTransmission(PCF8574_I2C_ADDRESS);
    if (Wire.endTransmission() == 0)
    {
        Serial.println("Found it.");
    }
    else
    {
        Serial.println("Not responding. Check wiring and the address pins,");
        Serial.println("or try 0x38 in code if this is a PCF8574A.");
    }

    printMenu();
}

void loop()
{
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    if (phUpOn && millis() - phUpOnAt >= PUMP_MAX_ON_MS)
    {
        Serial.println("[PH UP] auto-off, max on-time reached");
        turnOffPhUp();
    }
    if (phDownOn && millis() - phDownOnAt >= PUMP_MAX_ON_MS)
    {
        Serial.println("[PH DOWN] auto-off, max on-time reached");
        turnOffPhDown();
    }
}
