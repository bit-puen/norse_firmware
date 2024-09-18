#include "norsebot.h"
#include "config/pin.h"
#include "config/constant .h"
#include "esp_log.h"

static const char* TAG = "main_norsebot";

// #include "norseprotocol.h"

// dynamixel_config_t dynamixelConfig{
//   PIN_DYNAMIXEL_TX,         /* Tx pin */ 
//   PIN_DYNAMIXEL_RX,         /* Rx pin */ 
//   PIN_DYNAMIXEL_DIRECTION,  /* Direction pin */ 
//   BUADRATE_DYNAMIXEL        /* Buad rate */ 
// };
// protocol_config_t protocolConfig{
//   PIN_PROTOCOL_TX,          /* Tx pin */ 
//   PIN_PROTOCOL_RX,          /* Rx pin */ 
//   BUADRATE_PROTOCOL         /* Buad rate */ 
// };

// NorseBot norseBot(&dynamixelConfig, &protocolConfig);

// int main()
// {
//   ThisThread::sleep_for(2s);
//   norseBot.init();

//   while (true)
//   {
//     ThisThread::sleep_for(100ms);
//   }
  
//   return 0;
// }

// TaskHandle_t task;
// static void testThread(void *pvParamter);

HardwareSerial commandSerial(0);
HardwareSerial dynamixelSerial(1);
// NorseBot(commandSerial, dynamixelSerial, GPIO_NUM_4);
NorseBot norseBot(commandSerial, dynamixelSerial, GPIO_NUM_4);

// NorseProtocol testPTC(commandSerial, 115200);

void setup()
{
  ESP_LOGI(TAG, "Initial Norsebot");
  norseBot.init();
  delay(2000);
  // norseBot.autoModeHandler();
}
void loop()
{
  norseBot.updateControl();
  norseBot.updatePosition();
  delay(100);
}
