#ifndef NORSE_PROTOCOL_H
#define NORSE_PROTOCOL_H

#if PTC_MAJOR_VER == 1
#include "mbed.h"
#elif PTC_MAJOR_VER == 2
#include <Arduino.h>
#endif

#include "protocolconfig.h"

typedef struct
{
    uint8_t lineLength;
    uint8_t length;
    uint8_t eventId;
    uint8_t parameters[16];
} norse_packet_t;

class NorseProtocol
{
    public:
        #if defined(MK_I) || defined(MK_II)
        NorseProtocol(PinName tx, PinName rx, int baudRate);
        #elif defined(MK_III)
        NorseProtocol(HardwareSerial& serialPort, uint32_t baudRate);
        #endif

        void begin();

        norse_packet_t getPacket();
        bool getIsPacketAvilable();
        void runCommunication();

        void respondError(uint8_t reasonCode);
        void respondData(uint8_t dataRegister, uint8_t* data, uint8_t dataBytes);
        void respondOk(uint8_t eventId);

    private:
        void respondEventPing();

    private:
        #if PTC_MAJOR_VER == 1
        BufferedSerial _uart;
        #elif PTC_MAJOR_VER == 2
        HardwareSerial& _uart;
        #endif
        norse_packet_t _packet;

        volatile bool isPacketAvilable = true;
        bool isReading = false;
        uint8_t counter = 0;
        uint8_t errorIndex = 0x00;
        uint8_t lengthTxBuffer = 0;
        uint8_t lengthRxBuffer = 0;
        uint8_t rxBuffer[MAXIMUM_BUFFER_SIZE];          // Not optimal size
        uint8_t txBuffer[MAXIMUM_BUFFER_SIZE];          // Not optimal size
        // uint8_t parameters[MAXIMUM_BUFFER_SIZE/2];      // Not optimal size
        uint32_t _baudRate;

        void read();
        void write();
        void generatePacket(uint8_t eventId, uint8_t* params, uint8_t bytesParams);
        bool validateChecksum(uint8_t* packet, uint8_t bytes);
        bool validateHeader(); 
        uint16_t calculateChecksum(uint8_t* packet, uint8_t bytes);
        void eventHandler();
        
        #if PTC_MAJOR_VER == 1
        FileHandle* mbed_override_console(int fd);
        #endif
        
};

#endif