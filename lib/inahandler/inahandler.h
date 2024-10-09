#ifndef INA_HANDLER_H
#define INA_HANDLER_H

#include "Arduino.h"
#include "Adafruit_INA219.h"
#include "esp_log.h"

#define TAG_INA             "INA219"


class InaHandler
{
    public:
        InaHandler();
        ~InaHandler();

        void begin(uint8_t i2cAddress);
        float getVoltage();

    private:
        Adafruit_INA219* _ina;
};


#endif