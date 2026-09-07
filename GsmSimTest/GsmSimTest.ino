#include <SoftwareSerial.h>

// ============================================================
// ESP8266 ↔ SIM800L
// ------------------------------------------------------------
// ESP8266 D2 / GPIO4 = RX <- SIM800L TX
// ESP8266 D1 / GPIO5 = TX -> SIM800L RX
// ============================================================
SoftwareSerial sim800(D2, D1);

// ============================================================
// SETTINGS
// ============================================================
const unsigned long STATUS_INTERVAL = 15000; // 15 seconds

unsigned long lastStatusCheck = 0;

unsigned long testStart = 0;

unsigned int rdyCount = 0;
unsigned int callReadyCount = 0;
unsigned int smsReadyCount = 0;

// ============================================================
// PRINT ELAPSED TIME
// ============================================================
void printTime() {
  unsigned long seconds = (millis() - testStart) / 1000;

  unsigned long minutes = seconds / 60;
  seconds = seconds % 60;

  Serial.print("[");
  Serial.print(minutes);
  Serial.print("m ");
  Serial.print(seconds);
  Serial.print("s] ");
}

// ============================================================
// WATCH FOR IMPORTANT UNSOLICITED MESSAGES
// ============================================================
void inspectResponse(const String &response) {

  if (response.indexOf("RDY") >= 0) {
    rdyCount++;

    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("WARNING: MODEM RDY DETECTED");
    Serial.println("Possible SIM800L restart");
    Serial.print("RDY count: ");
    Serial.println(rdyCount);
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  if (response.indexOf("Call Ready") >= 0) {
    callReadyCount++;
  }

  if (response.indexOf("SMS Ready") >= 0) {
    smsReadyCount++;
  }
}

// ============================================================
// READ MODEM RESPONSE
// ============================================================
String readSIM(unsigned long timeout) {

  String response = "";

  unsigned long start = millis();

  while (millis() - start < timeout) {

    while (sim800.available()) {

      char c = sim800.read();

      response += c;
      Serial.write(c);
    }

    yield();
  }

  inspectResponse(response);

  return response;
}

// ============================================================
// SEND AT COMMAND
// ============================================================
String sendAT(const char *command, unsigned long timeout = 2500) {

  Serial.println();

  printTime();

  Serial.print(">>> ");
  Serial.println(command);

  sim800.println(command);

  return readSIM(timeout);
}

// ============================================================
// SYNCHRONIZE WITH SIM800L
// ============================================================
bool syncSIM800() {

  Serial.println();
  Serial.println("================================");
  Serial.println("SYNCING SIM800L");
  Serial.println("================================");

  for (int attempt = 1; attempt <= 15; attempt++) {

    Serial.print("AT attempt ");
    Serial.println(attempt);

    sim800.println("AT");

    String response = readSIM(2000);

    if (response.indexOf("OK") >= 0) {

      Serial.println();
      Serial.println("SIM800L SYNCED");

      return true;
    }

    delay(1000);
  }

  return false;
}

// ============================================================
// PRINT STATUS INTERPRETATION
// ============================================================
void explainCSQ(const String &response) {

  int pos = response.indexOf("+CSQ:");

  if (pos < 0) {
    return;
  }

  int comma = response.indexOf(',', pos);

  if (comma < 0) {
    return;
  }

  String valueString =
    response.substring(pos + 5, comma);

  valueString.trim();

  int rssi = valueString.toInt();

  Serial.print("Signal interpretation: ");

  if (rssi == 99) {
    Serial.println("UNKNOWN");
  }
  else if (rssi <= 9) {
    Serial.println("VERY WEAK");
  }
  else if (rssi <= 14) {
    Serial.println("USABLE");
  }
  else if (rssi <= 19) {
    Serial.println("GOOD");
  }
  else {
    Serial.println("VERY GOOD");
  }
}

// ============================================================
// EXPLAIN NETWORK REGISTRATION
// ============================================================
void explainCREG(const String &response) {

  int pos = response.indexOf("+CREG:");

  if (pos < 0) {
    return;
  }

  int comma = response.indexOf(',', pos);

  if (comma < 0) {
    return;
  }

  int end = response.indexOf('\r', comma);

  if (end < 0) {
    end = response.length();
  }

  String stateString =
    response.substring(comma + 1, end);

  stateString.trim();

  int state = stateString.toInt();

  Serial.print("Network interpretation: ");

  switch (state) {

    case 0:
      Serial.println("NOT REGISTERED");
      break;

    case 1:
      Serial.println("REGISTERED - HOME NETWORK");
      break;

    case 2:
      Serial.println("SEARCHING FOR NETWORK");
      break;

    case 3:
      Serial.println("REGISTRATION DENIED");
      break;

    case 4:
      Serial.println("REGISTRATION UNKNOWN");
      break;

    case 5:
      Serial.println("REGISTERED - ROAMING");
      break;

    default:
      Serial.println("UNKNOWN STATE");
      break;
  }
}

// ============================================================
// RUN ONE COMPLETE STATUS CHECK
// ============================================================
void runStatusCheck() {

  Serial.println();
  Serial.println();
  Serial.println("================================");
  Serial.println("STATUS CHECK");
  Serial.println("================================");

  // ----------------------------------------------------------
  // 1. INTERNAL VOLTAGE
  // ----------------------------------------------------------
  String cbc = sendAT("AT+CBC", 2500);

  // ----------------------------------------------------------
  // 2. SIGNAL
  // ----------------------------------------------------------
  String csq = sendAT("AT+CSQ", 2500);

  explainCSQ(csq);

  // ----------------------------------------------------------
  // 3. NETWORK REGISTRATION
  // ----------------------------------------------------------
  String creg = sendAT("AT+CREG?", 2500);

  explainCREG(creg);

  // ----------------------------------------------------------
  // COUNTERS
  // ----------------------------------------------------------
  Serial.println();
  Serial.println("--- EVENT COUNTERS ---");

  Serial.print("RDY: ");
  Serial.println(rdyCount);

  Serial.print("Call Ready: ");
  Serial.println(callReadyCount);

  Serial.print("SMS Ready: ");
  Serial.println(smsReadyCount);

  Serial.println("================================");
}

// ============================================================
// PROCESS UNSOLICITED MODEM OUTPUT
// ============================================================
void processUnsolicitedOutput() {

  if (!sim800.available()) {
    return;
  }

  String unsolicited = "";

  unsigned long start = millis();

  while (millis() - start < 1000) {

    while (sim800.available()) {

      char c = sim800.read();

      unsolicited += c;
      Serial.write(c);
    }

    yield();
  }

  inspectResponse(unsolicited);
}

// ============================================================
// SETUP
// ============================================================
void setup() {

  Serial.begin(115200);

  sim800.begin(9600);

  testStart = millis();

  Serial.println();
  Serial.println();
  Serial.println("================================");
  Serial.println("SIM800L POWER / NETWORK LOGGER");
  Serial.println("================================");

  Serial.println();
  Serial.println("Monitoring:");
  Serial.println("- AT+CBC");
  Serial.println("- AT+CSQ");
  Serial.println("- AT+CREG?");
  Serial.println("- RDY");
  Serial.println("- Call Ready");
  Serial.println("- SMS Ready");

  Serial.println();
  Serial.println("Waiting 5 seconds...");
  delay(5000);

  // ==========================================================
  // 1. SYNC MODEM
  // ==========================================================
  if (!syncSIM800()) {

    Serial.println();
    Serial.println("ERROR: FAILED TO SYNC WITH SIM800L");

    return;
  }

  // ==========================================================
  // 2. BASIC INFORMATION
  // ==========================================================
  sendAT("ATI", 3000);

  sendAT("AT+CFUN?", 3000);

  sendAT("AT+CSMINS?", 3000);

  sendAT("AT+CPIN?", 3000);

  // ==========================================================
  // 3. INITIAL STATUS
  // ==========================================================
  runStatusCheck();

  lastStatusCheck = millis();

  Serial.println();
  Serial.println();
  Serial.println("CONTINUOUS MONITOR STARTED");
  Serial.println("Status check every 15 seconds.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {

  // ----------------------------------------------------------
  // Always listen for unsolicited modem messages
  // ----------------------------------------------------------
  processUnsolicitedOutput();

  // ----------------------------------------------------------
  // Periodic CBC / CSQ / CREG test
  // ----------------------------------------------------------
  if (millis() - lastStatusCheck >= STATUS_INTERVAL) {

    lastStatusCheck = millis();

    runStatusCheck();
  }

  yield();
}