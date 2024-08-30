#include "norsebot.h"
#include "config/pin.h"
#include "config/constant .h"

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
  norseBot.init();
  // testPTC.begin();
  // xTaskCreatePinnedToCore(testThread, "protocolTask", 16384, NULL, 1, &task, 1);
}
void loop()
{
  // testPTC.runCommunication();
  delay(100);
}


// void testThread(void *pvParamter)
// {
//   bool isPacketAvilable;
//   printf("thread\r\n");
//     while (true)
//     {
//         testPTC.runCommunication();
//         isPacketAvilable = testPTC.getIsPacketAvilable();
//         if (isPacketAvilable)
//         {
//           testPTC.respondOk(testPTC.getPacket().eventId);
//         }
//     }
// }