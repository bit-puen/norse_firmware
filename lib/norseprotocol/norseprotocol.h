#include "mbed.h"
#include "protocolconfig.h"

struct NorsePacket
{
    uint8_t length;
    uint8_t eventId;
    uint8_t parameters[4];
};
typedef NorsePacket NorsePacket;

class NorseProtocol
{
    public:
        NorseProtocol(PinName tx, PinName rx, int baudRate);
        
        NorsePacket getPacket();
        bool getIsPacketAvilable();
        void runCommunication();

        void noticError(uint8_t reasonCode);

    private:
        void respondEventPing();

    private:
        BufferedSerial _uart;
        NorsePacket _packet;

        volatile bool isPacketAvilable = true;
        bool isReading = false;
        uint8_t counter = 0;
        uint8_t errorIndex = 0x00;
        uint8_t lengthTxBuffer = 0;
        uint8_t lengthRxBuffer = 0;
        uint8_t rxBuffer[MAXIMUM_BUFFER_SIZE];          // Not optimal size
        uint8_t txBuffer[MAXIMUM_BUFFER_SIZE];          // Not optimal size
        // uint8_t parameters[MAXIMUM_BUFFER_SIZE/2];      // Not optimal size
        int _baudRate;

        void read();
        void write();
        void generatePacket(uint8_t eventId, uint8_t *params, uint8_t bytesParams);
        bool validateChecksum(uint8_t *packet, uint8_t bytes);
        bool validateHeader(); 
        uint16_t calculateChecksum(uint8_t *packet, uint8_t bytes);
        FileHandle *mbed_override_console(int fd);
        void eventHandler();
};