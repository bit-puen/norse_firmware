#include "mbed.h"
#include "controltable.h"

enum StatusCode
{
    packetResultOk = 0x00,
    packetResultFail = 0x01,
    packetInstructionError = 0x02,
    packetCrcError = 0x03,
    packetDataRangeError = 0x04,
    packetDataLengthError = 0x05,
    packetDataLimitError = 0x06,
    packetAccessDenied = 0x07
};

////////// VARIABLE ///////////
#define DELAY_TIME_US               3000
#define DELAY_TX_OFFSET_US          6
#define DEFAULT_RESP_TIMEOUT_US     4000  // Estimate from oscilloscope @puen

class Dynamixel
{
    public:
        Dynamixel(PinName tx, PinName rx, uint32_t baudRate, PinName rs485DirectionPin);

        void enableTorque(uint8_t id);
        void disableTorque(uint8_t id);
        void enableLed(uint8_t id);
        void disableLed(uint8_t id);
        void setGoalVelocity(uint8_t id,uint16_t goalVelocity, uint8_t direction);
        int getPresentPosition(uint8_t id);

    private:
        BufferedSerial _dynamixel;
        DigitalOut _rs485DirectionPin;
        // Timer timeoutTimer;
        int _baudRate;
        int _responseTimeoutMicroSec;

        // void read(uint8_t id, uint16_t registerAddress, uint16_t bytes, uint8_t* data);
        StatusCode writeRegister(uint8_t id, uint16_t registerAddress, uint16_t bytesParams, uint8_t* data);
        void write(uint8_t id, uint8_t instruction, uint16_t registerAddress, uint16_t bytesParams, uint8_t* data);

        StatusCode readRegister(uint8_t id, uint16_t registerAddress, uint16_t bytes, int& data);
        StatusCode read(uint16_t bytesStuatusParams);
        StatusCode read(uint16_t bytesStuatusParams, int& data);
        unsigned short update_crc(unsigned short crc_accum, unsigned char* data_blk_prt, unsigned short data_blk_size);

};      