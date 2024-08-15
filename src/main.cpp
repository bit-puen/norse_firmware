#include "mbed.h"
#include "norsebot.h"
#include "norseprotocol.h"
#include "config/pin.h"

NorseProtocol nProtocol(USBTX, USBRX, 115200);
NorsePacket nPacket;
Thread readingThread;

volatile bool readingThreadEnable = true;

// static BufferedSerial serial_port(USBTX, USBRX, 115200);

// FileHandle *mbed::mbed_override_console(int fd)
// {
//   return &serial_port;
// }

void readMessage();

int main()
{
  // nProtocol.runCommunication();
  readingThread.start(readMessage);

  while (true)
  {
    // printf("test\r\n");
    ThisThread::sleep_for(100ms);
  }
  
  return 0;
}

void readMessage() 
{
  while (readingThreadEnable) {
    nProtocol.runCommunication();
    // isPacketAvilable = mainBoard.getIsPacketAvilable();
    if (nProtocol.getIsPacketAvilable()) 
    {
      nPacket = nProtocol.getPacket();
    }
    ThisThread::sleep_for(100ms);
  }
}