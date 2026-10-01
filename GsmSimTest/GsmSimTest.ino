#include <SoftwareSerial.h>

// ============================================================
// ESP8266 ↔ SIM800L V2
//
// Converted from the ESP32/HardwareSerial(UART1) version - ESP8266 has no
// full-duplex spare hardware UART available alongside USB debug Serial
// (UART0 is shared with USB; UART1 is TX-only, no RX), so this uses the
// same EspSoftwareSerial library already vendored in this repo (see
// BasilienceFirmware_V2/libraries/EspSoftwareSerial) instead.
//
// ESP8266 D2 / GPIO4 = RX <- SIM800L TXD
// ESP8266 D1 / GPIO5 = TX -> SIM800L RXD
// (Same D2/D1 pins this sketch's original ESP8266 version used, before it
// was adapted to the ESP32's GPIO36/GPIO23 for the Basilience pin audit.)
// ============================================================

constexpr uint8_t SIM800_RX = D2;
constexpr uint8_t SIM800_TX = D1;

SoftwareSerial sim800l(SIM800_RX, SIM800_TX);

constexpr unsigned long PC_BAUD  = 115200;
constexpr unsigned long GSM_BAUD = 9600;

// Change this only if you want another recipient.
const char TARGET_NUMBER[] = "+639658904777";

const char TEST_MESSAGE[] =
    "Hello from ESP8266 Basilience GSM test";

// ============================================================
// AT RESPONSE
// ============================================================

struct ATResponse {
  String text;
  bool ok;
  bool error;
  bool timeout;
};

// ============================================================
// READ UART
// ============================================================

ATResponse readResponse(
    unsigned long timeout,
    bool stopAtPrompt = false) {

  ATResponse result;

  result.text = "";
  result.ok = false;
  result.error = false;
  result.timeout = false;

  unsigned long started = millis();

  while (millis() - started < timeout) {

    while (sim800l.available()) {

      char c = sim800l.read();

      result.text += c;
      Serial.write(c);

      // SMS message-entry prompt
      if (stopAtPrompt && c == '>') {
        return result;
      }

      if (
        result.text.indexOf("\r\nOK\r\n") >= 0 ||
        result.text.endsWith("OK\r\n")
      ) {
        result.ok = true;
        return result;
      }

      if (
        result.text.indexOf("+CMS ERROR:") >= 0 ||
        result.text.indexOf("+CME ERROR:") >= 0 ||
        result.text.indexOf("\r\nERROR\r\n") >= 0
      ) {
        result.error = true;
        return result;
      }
    }

    delay(1);
  }

  result.timeout = true;

  return result;
}

// ============================================================
// CLEAR OLD UART DATA
// ============================================================

void clearInput() {

  while (sim800l.available()) {
    Serial.write(sim800l.read());
  }
}

// ============================================================
// SEND AT COMMAND
// ============================================================

ATResponse sendAT(
    const String &command,
    unsigned long timeout = 5000) {

  clearInput();

  Serial.println();
  Serial.print(">>> ");
  Serial.println(command);

  sim800l.println(command);

  return readResponse(timeout);
}

// ============================================================
// SYNCHRONIZE SIM800L
// ============================================================

bool syncModem() {

  Serial.println();
  Serial.println("================================");
  Serial.println("SYNCING SIM800L");
  Serial.println("================================");

  for (int attempt = 1; attempt <= 15; attempt++) {

    Serial.print("Attempt ");
    Serial.print(attempt);
    Serial.println("/15");

    clearInput();

    sim800l.println("AT");

    ATResponse response =
        readResponse(2500);

    if (response.ok) {

      Serial.println();
      Serial.println("SIM800L CONNECTED");
      return true;
    }

    delay(1000);
  }

  Serial.println();
  Serial.println("SIM800L CONNECTION FAILED");

  return false;
}

// ============================================================
// PARSE CREG STATE
// ============================================================

int parseCREG(const String &response) {

  int tag = response.indexOf("+CREG:");

  if (tag < 0) {
    return -1;
  }

  int comma = response.indexOf(',', tag);

  if (comma < 0) {
    return -1;
  }

  int end = response.indexOf('\r', comma);

  if (end < 0) {
    end = response.length();
  }

  String value =
      response.substring(comma + 1, end);

  value.trim();

  return value.toInt();
}

// ============================================================
// CHECK SIM
// ============================================================

bool simReady() {

  ATResponse response =
      sendAT("AT+CPIN?", 5000);

  return
      response.text.indexOf("+CPIN: READY") >= 0;
}

// ============================================================
// CHECK NETWORK
// ============================================================

bool networkRegistered() {

  ATResponse response =
      sendAT("AT+CREG?", 5000);

  int state = parseCREG(response.text);

  return state == 1 || state == 5;
}

// ============================================================
// STATUS
// ============================================================

void showStatus() {

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP8266 + SIM800L STATUS");
  Serial.println("================================");

  sendAT("ATI");

  sendAT("AT+CFUN?");

  sendAT("AT+CSMINS?");

  ATResponse pin =
      sendAT("AT+CPIN?");

  Serial.println();

  if (pin.text.indexOf("+CPIN: READY") >= 0) {
    Serial.println("SIM: READY");
  } else {
    Serial.println("SIM: NOT READY");
  }

  sendAT("AT+CCID");

  sendAT("AT+CIMI");

  sendAT("AT+CBC");

  sendAT("AT+CSQ");

  ATResponse creg =
      sendAT("AT+CREG?");

  int state =
      parseCREG(creg.text);

  Serial.println();
  Serial.print("NETWORK: ");

  switch (state) {

    case 0:
      Serial.println("NOT REGISTERED");
      break;

    case 1:
      Serial.println("REGISTERED - HOME");
      break;

    case 2:
      Serial.println("SEARCHING");
      break;

    case 3:
      Serial.println("DENIED");
      break;

    case 4:
      Serial.println("UNKNOWN");
      break;

    case 5:
      Serial.println("REGISTERED - ROAMING");
      break;

    default:
      Serial.println("NO VALID RESPONSE");
      break;
  }

  sendAT("AT+COPS?", 8000);

  Serial.println();
  Serial.println("================================");
}

// ============================================================
// NETWORK REGISTRATION
// ============================================================

void connectNetwork() {

  Serial.println();
  Serial.println("================================");
  Serial.println("NETWORK REGISTRATION");
  Serial.println("================================");

  if (!simReady()) {

    Serial.println();
    Serial.println("SIM IS NOT READY.");
    return;
  }

  Serial.println();
  Serial.println("Selecting operator automatically...");

  sendAT("AT+COPS=0", 30000);

  Serial.println();
  Serial.println("Waiting for registration...");

  unsigned long started = millis();

  while (millis() - started < 90000) {

    ATResponse response =
        sendAT("AT+CREG?", 5000);

    int state =
        parseCREG(response.text);

    if (state == 1) {

      Serial.println();
      Serial.println("NETWORK REGISTERED - HOME");

      sendAT("AT+COPS?", 8000);
      sendAT("AT+CSQ");

      return;
    }

    if (state == 5) {

      Serial.println();
      Serial.println("NETWORK REGISTERED - ROAMING");

      sendAT("AT+COPS?", 8000);
      sendAT("AT+CSQ");

      return;
    }

    Serial.print("Waiting. CREG state = ");
    Serial.println(state);

    delay(5000);
  }

  Serial.println();
  Serial.println("NETWORK REGISTRATION TIMEOUT");
}

// ============================================================
// SEND SMS
// ============================================================

void sendSMS() {

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP8266 SMS TEST");
  Serial.println("================================");

  // ----------------------------------------------------------
  // SIM must be ready
  // ----------------------------------------------------------

  if (!simReady()) {

    Serial.println();
    Serial.println("SMS CANCELLED: SIM NOT READY");

    return;
  }

  // ----------------------------------------------------------
  // Must already be registered
  // ----------------------------------------------------------

  if (!networkRegistered()) {

    Serial.println();
    Serial.println("SMS CANCELLED: NETWORK NOT REGISTERED");
    Serial.println("Run NETWORK first.");

    return;
  }

  // ----------------------------------------------------------
  // Diagnostic signal
  // ----------------------------------------------------------

  sendAT("AT+CSQ");

  // ----------------------------------------------------------
  // Enable detailed errors
  // ----------------------------------------------------------

  sendAT("AT+CMEE=2");

  // ----------------------------------------------------------
  // Text SMS mode
  // ----------------------------------------------------------

  ATResponse textMode =
      sendAT("AT+CMGF=1");

  if (!textMode.ok) {

    Serial.println();
    Serial.println("SMS FAILED: TEXT MODE REJECTED");

    return;
  }

  // ----------------------------------------------------------
  // Read existing SMSC.
  // DO NOT overwrite it.
  // ----------------------------------------------------------

  sendAT("AT+CSCA?");

  // ----------------------------------------------------------
  // Enter recipient
  // ----------------------------------------------------------

  clearInput();

  Serial.println();
  Serial.print(">>> AT+CMGS=\"");
  Serial.print(TARGET_NUMBER);
  Serial.println("\"");

  sim800l.print("AT+CMGS=\"");
  sim800l.print(TARGET_NUMBER);
  sim800l.println("\"");

  // ----------------------------------------------------------
  // Wait for >
  // ----------------------------------------------------------

  ATResponse prompt =
      readResponse(15000, true);

  if (prompt.text.indexOf('>') < 0) {

    Serial.println();
    Serial.println("SMS FAILED: NO > PROMPT");

    return;
  }

  // ----------------------------------------------------------
  // Message body
  // ----------------------------------------------------------

  Serial.println();
  Serial.print("Sending: ");
  Serial.println(TEST_MESSAGE);

  sim800l.print(TEST_MESSAGE);

  delay(500);

  // Ctrl+Z
  sim800l.write(26);

  Serial.println();
  Serial.println("CTRL+Z sent.");
  Serial.println("Waiting for +CMGS...");

  // ----------------------------------------------------------
  // Wait for network SMS result
  // ----------------------------------------------------------

  String response = "";

  unsigned long started = millis();

  while (millis() - started < 60000) {

    while (sim800l.available()) {

      char c = sim800l.read();

      response += c;
      Serial.write(c);
    }

    if (
      response.indexOf("+CMGS:") >= 0 &&
      response.indexOf("OK") >= 0
    ) {

      Serial.println();
      Serial.println();
      Serial.println("================================");
      Serial.println("SMS SENT SUCCESSFULLY");
      Serial.println("================================");

      return;
    }

    if (
      response.indexOf("+CMS ERROR:") >= 0 ||
      response.indexOf("+CME ERROR:") >= 0 ||
      response.indexOf("\r\nERROR\r\n") >= 0
    ) {

      Serial.println();
      Serial.println();
      Serial.println("================================");
      Serial.println("SMS SEND FAILED");
      Serial.println("================================");

      return;
    }

    delay(1);
  }

  Serial.println();
  Serial.println("SMS SEND TIMEOUT");
}

// ============================================================
// HELP
// ============================================================

void showHelp() {

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP8266 + SIM800L TEST CONSOLE");
  Serial.println("================================");

  Serial.println("STATUS  - modem/SIM/network status");
  Serial.println("NETWORK - automatic GSM registration");
  Serial.println("SEND    - send test SMS");
  Serial.println("HELP    - display this menu");
  Serial.println();
  Serial.println("Raw AT commands also work.");
  Serial.println("================================");
}

// ============================================================
// USER COMMAND
// ============================================================

void processCommand(String command) {

  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command.equalsIgnoreCase("STATUS")) {
    showStatus();
    return;
  }

  if (command.equalsIgnoreCase("NETWORK")) {
    connectNetwork();
    return;
  }

  if (command.equalsIgnoreCase("SEND")) {
    sendSMS();
    return;
  }

  if (command.equalsIgnoreCase("HELP")) {
    showHelp();
    return;
  }

  String upper = command;
  upper.toUpperCase();

  if (upper.startsWith("AT")) {
    sendAT(command, 15000);
    return;
  }

  Serial.println("UNKNOWN COMMAND");
  Serial.println("Type HELP.");
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(PC_BAUD);

  // SoftwareSerial on the ESP8266's D2(RX)/D1(TX) - see this file's top
  // comment for why a software link is used here instead of a hardware
  // UART (ESP8266 has no spare full-duplex hardware UART alongside USB
  // debug Serial).
  sim800l.begin(GSM_BAUD);

  Serial.println();
  Serial.println();
  Serial.println("================================");
  Serial.println("BASILIENCE ESP8266 GSM VALIDATION");
  Serial.println("================================");
  Serial.println("UART RX: D2 (GPIO4)");
  Serial.println("UART TX: D1 (GPIO5)");
  Serial.println("SIM800L: 9600 baud");

  delay(5000);

  if (!syncModem()) {

    Serial.println();
    Serial.println("STOPPED: SIM800L NOT RESPONDING");

    return;
  }

  // Cleaner responses
  sendAT("ATE0");

  showHelp();
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // Unsolicited modem messages
  while (sim800l.available()) {
    Serial.write(sim800l.read());
  }

  // PC command input
  if (Serial.available()) {

    String command =
        Serial.readStringUntil('\n');

    processCommand(command);
  }

  delay(1);
}
