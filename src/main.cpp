#include "norsebot.h"
#include "config/pin.h"
#include "config/constant .h"
#include "esp_log.h"

static const char* TAG = "main_norsebot";

HardwareSerial commandSerial(0);
HardwareSerial dynamixelSerial(1);
NorseBot norseBot(commandSerial, 
                  dynamixelSerial, 
                  PIN_DYNAMIXEL_DIRECTION,
                  PIN_OBSTACLE_DETECTION);

void setup()
{
  ESP_LOGI(TAG, "Initial Norsebot");
  norseBot.init();
  delay(2000);
}

void loop()
{
  norseBot.updateControl();
  norseBot.updatePosition();
  norseBot.updateObstacle();
  norseBot.updateTail();
  delay(100);
}
