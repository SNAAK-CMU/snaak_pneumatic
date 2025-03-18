#include "ClearCore.h"

#define InputConnector ConnectorIO2
#define ENABLE_VACUUM_PIN IO0
#define EJECT_VACUUM_PIN IO1
#define ENABLE_BTN_PIN IO2
#define EJECT_BTN_PIN IO3

#define baudRateSerialPort 115200
#define baudRateInputPort 115200
#define isTtlInputPort false

bool enableButtonValue;
PinStatus ejectButtonValue;

bool isEnable = false;
int input;

void setup() {
  // Initialize Serial
  Serial.begin(baudRateSerialPort); // Initialize serial communication with ROS2
  while (!Serial) {
    continue;
  }

  Serial.ttl(isTtlInputPort);
  while (!Serial0) {
    continue;
  }

  // Define inputs and outputs
  pinMode(ENABLE_VACUUM_PIN, OUTPUT);
  pinMode(EJECT_VACUUM_PIN, OUTPUT);
  pinMode(ENABLE_BTN_PIN, INPUT);
  pinMode(EJECT_BTN_PIN, INPUT);

  // Reset Pneumatic Ports
  digitalWrite(ENABLE_VACUUM_PIN, LOW);
  digitalWrite(EJECT_VACUUM_PIN, LOW);
}

void loop() {
  // Read Buttons Values
  enableButtonValue = InputConnector.InputRisen();
  ejectButtonValue = digitalRead(EJECT_BTN_PIN);

  // Read Serial
  input = Serial.read();

  if (input != -1) {
    String command = "";
    while (Serial.available()) {
      command += (char)input;
      input = Serial.read();
    }
    
    if (command == "enable" || isEnable) {
      isEnable = true;
      digitalWrite(ENABLE_VACUUM_PIN, HIGH);
    } 
    if (isEnable && command == "disable") {
      isEnable = false;
      digitalWrite(ENABLE_VACUUM_PIN, LOW);
    } 
    if (command.startsWith("eject")) {
      int duration = command.substring(5).toInt();
      digitalWrite(EJECT_VACUUM_PIN, HIGH);
      delay(duration);
      digitalWrite(EJECT_VACUUM_PIN, LOW);
    }
  }
  
  delay(10);
}
