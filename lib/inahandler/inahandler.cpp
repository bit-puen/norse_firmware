#include "inahandler.h"

InaHandler::InaHandler()
{
    _ina = new Adafruit_INA219();
}

InaHandler::~InaHandler()
{
}

void InaHandler::begin(uint8_t i2cAddress)
{
    if (!_ina->begin())
    {
        ESP_LOGI(TAG_INA, "Failed to find INA219 chip");
        while (1) { delay(10); }
    }
    ESP_LOGI(TAG_INA, "Found INA219 chip");
}

float InaHandler::getVoltage()
{
    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;

    shuntvoltage = _ina->getShuntVoltage_mV();
    busvoltage = _ina->getBusVoltage_V();
    current_mA = _ina->getCurrent_mA();
    power_mW = _ina->getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    return loadvoltage;
}
