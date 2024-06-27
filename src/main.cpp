#include <Arduino.h>
#include <ESP32Servo.h>
#include "config/pin.h"
// #include "servo_pattern.h"
// #define SERVO_DEG_PER_MS    0.857142857  // D2S51
// #define SERVO_DEG_PER_MS    0.75  // D2S51
// #define SERVO_DEG_PER_MS    0.923076923  // D2S51
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
