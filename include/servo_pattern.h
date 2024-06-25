#include <Arduino.h>
#include <ESP32Servo.h>

/*
* SG90, 0.12s/60deg --> 0.002s/1deg
* 
*/
void delayPattern(uint16_t delayTime)
{
    unsigned long startTime = millis();
    while (millis() - startTime < delayTime)
    {
        vTaskDelay(1);
    }
}

void pattern1(Servo servo)
{
    // -90, 180ms
    servo.write(0);
    delayPattern(180);
    // 0, 180ms
    servo.write(90);
    delayPattern(180);
    // +90, 180ms
    servo.write(180);
    delayPattern(180);
    // +360, 720ms
    servo.write(180);
    delayPattern(720);
    
    servo.write(90);
}