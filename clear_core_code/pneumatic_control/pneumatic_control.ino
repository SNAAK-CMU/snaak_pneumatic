#include "ClearCore.h"

#define InputConnector1 ConnectorIO2
#define ENABLE_VACUUM_PIN IO0
#define EJECT_VACUUM_PIN IO1
#define ENABLE_BTN_PIN IO2
#define EJECT_BTN_PIN IO3

#define baudRateSerialPort 115200
#define baudRateInputPort 115200
#define isTtlInputPort false

bool toggleVacuum;
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
  toggleVacuum = InputConnector1.InputRisen();
  ejectButtonValue = digitalRead(EJECT_BTN_PIN);

  // Read Serial
  input = Serial.read();
  String command = "";

  // Convert Serial to commands: enable, disable and eject
  if (input != -1) {
    while (Serial.available()) {
      command += (char)input;
      input = Serial.read();
    }
  }

  // Enable Vacuum
  if ((command == "enable") || (toggleVacuum == true) || (isEnable)) {
    isEnable = true;
    toggleVacuum = false;
    digitalWrite(ENABLE_VACUUM_PIN, HIGH);
  } 

  // Disable Vacuum
  if ( ((isEnable) && (command == "disable"))  ) {
    isEnable = false;
    toggleVacuum = false;
    digitalWrite(ENABLE_VACUUM_PIN, LOW);
  } 

  // Eject Vacuum
  if ((command.startsWith("eject")) || (ejectButtonValue == true)) {
    int duration = command.substring(5).toInt();
    duration = max(duration, 500);

    isEnable = false;
    digitalWrite(ENABLE_VACUUM_PIN, LOW);
    
    digitalWrite(EJECT_VACUUM_PIN, HIGH);
    delay(duration);
    digitalWrite(EJECT_VACUUM_PIN, LOW);
  }

  
  delay(10);
}
