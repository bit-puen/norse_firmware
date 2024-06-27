#include <Arduino.h>
#include <ESP32Servo.h>
#include "config/pin.h"
// #include "servo_pattern.h"

Servo turnTable;

void pattern1();

void setup() {
  Serial.begin(115200);
  turnTable.attach(PIN_SERVO);
  Serial.println("Hello world!");
}

void loop() 
{
  pattern1();
  delay(1000);
}


void pattern1()
{
    // -90, 180ms
    turnTable.writeMicroseconds(900);
    delay(150);

    // 0, 180ms
    turnTable.writeMicroseconds(1500);
    delay(150);

    // +90, 180ms
    turnTable.writeMicroseconds(2100);
    delay(150);

    // 0, 180ms
    turnTable.writeMicroseconds(1500);
    delay(150);

    // +90, 180ms
    turnTable.writeMicroseconds(2100);
    delay(150);

    // 0, 180ms
    turnTable.writeMicroseconds(1500);
    delay(150);

    // -90, 180ms
    turnTable.writeMicroseconds(900);
    delay(150);

    // 0, 180ms
    turnTable.writeMicroseconds(1500);
    delay(150);

    // +360, 720ms
    turnTable.writeMicroseconds(2100);
    delay(420);
    
    // 0, 180ms
    turnTable.write(1500);
    delay(150);
}