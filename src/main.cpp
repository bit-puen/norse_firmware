#include <Arduino.h>
#include <ESP32Servo.h>
#include "config/pin.h"
#include "CytronMotorDriver.h"


CytronMD motor(PWM_DIR, PIN_DRIVER_PWM, PIN_DRIVER_DIR);
uint8_t directionState = 0;
uint8_t syringeState = 0;

void setup() 
{
  Serial.begin(115200);
  Serial.println("Hello world!");

  pinMode(PIN_DIRECTION, INPUT);
  pinMode(PIN_SYRINGE, INPUT);

  delay(1000);

  // motor.setSpeed(128);  // Run forward at 50% speed.
  // delay(1000);
  // motor.setSpeed(0);    // Stop.
  // delay(1000);
  // motor.setSpeed(-128);  // Run backward at 50% speed.
  // delay(1000);
  // motor.setSpeed(0);    // Stop.
  // delay(1000);
}

void loop() 
{
  directionState = digitalRead(PIN_DIRECTION);
  syringeState = digitalRead(PIN_SYRINGE);

  Serial.printf("DIR: %d \t SYR: %d \r\n", directionState, syringeState);
  
  if (( directionState == HIGH ) && ( syringeState == HIGH ))
  {
    motor.setSpeed(-255);  // Run backward at 50% speed.
  }
  else if (( directionState == LOW ) && ( syringeState == HIGH ))
  {
    motor.setSpeed(32);  // Run forward at 50% speed.
  }
  else
  {
    motor.setSpeed(0);    // Stop.
  }
  
  delay(50);
}

/*
#define SERVO_DEG_PER_MS    1.2  // SG90
#define SERVO_MAX_MSTIME    600
#define SERVO_MID_MSTIME    1500


Servo turnTable;


void moveCw90(uint16_t intervalTime, uint16_t degree);
void moveCcw90(uint16_t intervalTime, uint16_t degree);
void pattern1();
void pattern2();

void setup() {
  Serial.begin(115200);
  turnTable.attach(PIN_SERVO);
  Serial.println("Hello world!");
}

void loop() 
{
  // pattern1();
  pattern2();
  delay(2000);
}

void moveCw90(uint16_t intervalTime, uint16_t degree)
{
  float goalDegPerMs = (float)degree / (float)intervalTime;

  // Serial.print("goalDegPerMs: ");
  // Serial.println(goalDegPerMs);
  // Serial.println(intervalTime);

  int msPwm = SERVO_MID_MSTIME - int((SERVO_MAX_MSTIME / SERVO_DEG_PER_MS) * goalDegPerMs);
  // uint16_t startTime = millis();

  // Serial.print("msPwm: ");
  // Serial.println(msPwm);
  turnTable.writeMicroseconds(msPwm);
  delay(intervalTime);
  turnTable.writeMicroseconds(1500);
}

void moveCcw90(uint16_t intervalTime, uint16_t degree)
{
  float goalDegPerMs = (float)degree / (float)intervalTime;
  int msPwm = SERVO_MID_MSTIME + int((SERVO_MAX_MSTIME / SERVO_DEG_PER_MS) * goalDegPerMs);

  turnTable.writeMicroseconds(msPwm);
  delay(intervalTime);
  turnTable.writeMicroseconds(1500);
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

void pattern2()
{
  moveCw90(150, 90);
  delay(150);
  moveCcw90(150, 90);
  delay(150);
  moveCcw90(150, 90);
  delay(150);
  moveCw90(150, 90);
  delay(150);
  moveCcw90(1000, 360); 
  moveCcw90(222, 80); // +error
  delay(150);
}
*/