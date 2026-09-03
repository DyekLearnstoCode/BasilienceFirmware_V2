#include <SoftwareSerial.h>

SoftwareSerial sim800(4, 5); // RX, TX

String sendCommand(String command, unsigned long timeout = 1000) {
  while (sim800.available()) {
    sim800.read();
  }

  sim800.println(command);

  String response = "";
  unsigned long start = millis();

  while (millis() - start < timeout) {
    while (sim800.available()) {
      response += (char)sim800.read();
    }

    yield();
  }

  return response;
}

void setup() {
  Serial.begin(115200);
  sim800.begin(9600);

  delay(5000);

  Serial.println();
  Serial.println("SIM800L SIM CONTACT TEST");
  Serial.println("------------------------");

  Serial.println(sendCommand("AT"));
  Serial.println(sendCommand("AT+CSMINS=1"));
}

void loop() {

  Serial.println();
  Serial.println("==========================");

  String simStatus = sendCommand("AT+CSMINS?");
  Serial.print("SIM DETECTION:");
  Serial.println(simStatus);

  String pinStatus = sendCommand("AT+CPIN?");
  Serial.print("SIM COMMUNICATION:");
  Serial.println(pinStatus);

  Serial.println("==========================");

  delay(2000);
}