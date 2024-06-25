#include <Arduino.h>
#include <ESP32Servo.h>
#include "config/pin.h"
#include "servo_pattern.h"

Servo turnTable;

void setup() {
  turnTable.attach(PIN_SERVO);

  pattern1(turnTable);
}

void loop() {
  // put your main code here, to run repeatedly:
}
