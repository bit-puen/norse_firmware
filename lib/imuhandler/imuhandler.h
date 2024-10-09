#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "Arduino.h"
#include "Adafruit_BNO08x.h"
#include "esp_log.h"

#define IMU_RESET       -1
#define TAG_IMU         "IMU"

class ImuHandler
{
    public:
        ImuHandler();
        ~ImuHandler();

        void begin(uint8_t i2cAddress);

    private:
        Adafruit_BNO08x* _imu;

};

#endif