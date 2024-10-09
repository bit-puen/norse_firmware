#include "imuhandler.h"

ImuHandler::ImuHandler()
{
    _imu = new Adafruit_BNO08x(IMU_RESET);
}

ImuHandler::~ImuHandler()
{
}

void ImuHandler::begin(uint8_t i2cAddress)
{
    if (!_imu->begin_I2C(i2cAddress))
    {
        ESP_LOGI(TAG_IMU, "Failed to find BNO08x chip");
        while (1) { delay(100); }
        // while (_imu->begin_I2C(i2cAddress)) { delay(100); }
    }
    ESP_LOGI(TAG_IMU, "Found BNO08x chip");
}