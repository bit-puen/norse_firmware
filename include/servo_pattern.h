#include <Arduino.h>
#include <ESP32Servo.h>

/*
* SG90,     0.12s/60deg --> 0.002s/1deg
* D2S51,    0.07s/60deg --> 0.001166667s/1deg
* D2S51,    0.05s/60deg --> 0.000833333s/1deg
*/
void delayPattern(uint16_t delayTime)
{
    unsigned long startTime = millis();
    while (millis() - startTime < delayTime)
    {
        delay(1);
    }
}


void pattern1(Servo servo)
{
    // -90, 180ms
    servo.writeMicroseconds(900);
    delay(150);

    // 0, 180ms
    servo.writeMicroseconds(1500);
    delay(150);

    // +90, 180ms
    servo.writeMicroseconds(2100);
    delay(150);

    // 0, 180ms
    servo.writeMicroseconds(1500);
    delay(150);

    // +90, 180ms
    servo.writeMicroseconds(2100);
    delay(150);

    // 0, 180ms
    servo.writeMicroseconds(1500);
    delay(150);

    // -90, 180ms
    servo.writeMicroseconds(900);
    delay(150);

    // 0, 180ms
    servo.writeMicroseconds(1500);
    delay(150);

    // +360, 720ms
    servo.writeMicroseconds(2100);
    delay(420);
    
    // 0, 180ms
    servo.write(1500);
    delay(150);
}