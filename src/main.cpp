#include <Arduino.h>
#include <ESP32Servo.h>
#include "config/pin.h"
#include "servo_pattern.h"

Servo turnTable;

void setup() {
  Serial.begin(115200);
  turnTable.attach(PIN_SERVO);
  Serial.println("Hello world!");
  pattern1(turnTable);
}

void loop() 
{
}
