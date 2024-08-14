#include "mbed.h"

////////// EEPROM ///////////

////////// RAM ///////////
#define REG_TORQUE_ENABLE       0x40
#define REG_LED_ENABLE          0x41
#define REG_GOAL_VELOCITY       0x68
#define REG_PRESENT_VELOCITY    0x80

////////// INSTRUCTION ///////////
#define PING                    0x01
#define READ_DATA               0x02
#define WRITE_DATA              0x03
#define REBOOT                  0x08

////////// ERROR ///////////
#define RESULT_FAIL             0x01
#define INS_ERR                 0x02
#define CRC_ERR                 0x03
#define DATA_RANGE_ERR          0x04
#define DATA_LENGTH_ERR         0x05
#define DATA_LIMIT_ERR          0x06
#define ACCESS_ERR              0x07

////////// VARIABLE ///////////
#define DELAY_TIME_US           3000

class Dynamixel
{
    public:
        Dynamixel(PinName tx, PinName rx, uint32_t baudRate, PinName rs485DirectionPin);

        void enableTorque(uint8_t id);
        void disableTorque(uint8_t id);
        void enableLed(uint8_t id);
        void disableLed(uint8_t id);
        void setGoalVelocity(uint8_t id,uint16_t goalVelocity, uint8_t direction);
        

    private:
        BufferedSerial _dynamixel;
        DigitalOut _rs485DirectionPin;
        int _baudRate;
        // void read(uint8_t id, uint16_t registerAddress, uint16_t bytes, uint8_t* data);
        void write(uint8_t id, uint16_t registerAddress, uint16_t bytes, uint8_t* data);
        unsigned short update_crc(unsigned short crc_accum, unsigned char* data_blk_prt, unsigned short data_blk_size);

};      