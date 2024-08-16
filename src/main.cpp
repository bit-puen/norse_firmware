#include "mbed.h"
#include "norsebot.h"
#include "config/pin.h"
#include "config/instance.h"


dynamixel_config_t dynamixelConfig{
  PIN_DYNAMIXEL_TX,         /* Tx pin */ 
  PIN_DYNAMIXEL_RX,         /* Rx pin */ 
  PIN_DYNAMIXEL_DIRECTION,  /* Direction pin */ 
  BUADRATE_PROTOCOL         /* Buad rate */ 
};
protocol_config_t protocolConfig{
  PIN_PROTOCOL_TX,          /* Tx pin */ 
  PIN_PROTOCOL_RX,          /* Rx pin */ 
  BUADRATE_PROTOCOL         /* Buad rate */ 
};

NorseBot norseBot(&dynamixelConfig, &protocolConfig);

int main()
{
  norseBot.init();

  while (true)
  {
    ThisThread::sleep_for(100ms);
  }
  
  return 0;
}
