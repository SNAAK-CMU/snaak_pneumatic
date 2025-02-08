#include "ClearCore.h"

#define ENABLE_VACUUM_PIN IO0
#define EJECT_VACCUM_PIN IO1
#define ENABLE_BTN_PIN IO2
#define EJECT_BTN_PIN IO3

#define InputConnector ConnectorIO2

bool enableButtonValue;
PinStatus ejectButtonValue;

bool isEnable = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENABLE_VACUUM_PIN, OUTPUT);
  pinMode(EJECT_VACCUM_PIN, OUTPUT);

  pinMode(ENABLE_BTN_PIN, INPUT);
  pinMode(EJECT_BTN_PIN, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  enableButtonValue = InputConnector.InputRisen();
  ejectButtonValue = digitalRead(EJECT_BTN_PIN);

  if((isEnable == false) && (enableButtonValue == true)){
    isEnable = true;
    enableButtonValue = 0;

  }

  if((isEnable == true) && (enableButtonValue == true)){
    isEnable = false;
    enableButtonValue = 0;
  }

  if(isEnable){
    digitalWrite(ENABLE_VACUUM_PIN, HIGH);
  } else {
    digitalWrite(ENABLE_VACUUM_PIN, LOW);
  }


  if(ejectButtonValue){
    digitalWrite(EJECT_VACCUM_PIN, HIGH);
  } else {
    digitalWrite(EJECT_VACCUM_PIN, LOW);
  }

  delay(100);
}
