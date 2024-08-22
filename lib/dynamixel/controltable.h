/*************************************
    EEPROM
*************************************/


/*************************************
    RAM
*************************************/
#define REG_TORQUE_ENABLE       0x40
#define LEN_TORQUE_ENABLE       1
#define REG_LED_ENABLE          0x41
#define LEN_LED_ENABLE          1
#define REG_GOAL_VELOCITY       0x68
#define LEN_GOAL_VELOCITY       4
#define REG_PRESENT_VELOCITY    0x80
#define LEN_PRESENT_VELOCITY    4
#define REG_PRESENT_POSITION    0x84
#define LEN_PRESENT_POSITION    4

/*************************************
    INSTRUCTION
*************************************/
#define INSTRUCTION_PING        0x01
#define INSTRUCTION_READ_DATA   0x02
#define INSTRUCTION_WRITE_DATA  0x03
#define INSTRUCTION_REBOOT      0x08