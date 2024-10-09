#ifndef CONFIG_CONSTANT_H
#define CONFIG_CONSTANT_H

#include "protocolconfig.h"

#if defined(MK_I) || defined(MK_II)
#define BUADRATE_DYNAMIXEL          1000000
#elif defined(MK_III)
#define BUADRATE_DYNAMIXEL          115200
#define DYNAMIXEL_PROTOCOL_VERSION  2.0
#endif
#define BUADRATE_PROTOCOL           115200

#define WHEEL_FRONT_RIGHT_ID        1
#define WHEEL_FRONT_LEFT_ID         2
#define WHEEL_REAR_LEFT_ID          3
#define WHEEL_REAR_RIGHT_ID         4
#define TAIL_ID                     5

#define LEN_WHEEL_CENTER_X          0.119
#define LEN_WHEEL_CENTER_Y          0.119
#define WHEEL_RADIUS                0.03

#define I2C_ADDRESS_INA219          0x40
#define I2C_ADDRESS_BNO085          0x4A
#define MOTOR_MAX_RPM               55

/* Default Status */
#define DFT_CONTROL_MODE            PARAM_MANUAL_DRIVE_MODE
#define DFT_MANUAL_SPEED            0
#define DFT_MANUAL_CMD              PARAM_MOVING_ST
#define DFT_TAIL_MODE               PARAM_TAIL_POS_CONTROL
#define DFT_TAIL_RANGE              60
#define DFT_TAIL_LOW_SPEED          442
#define DFT_TAIL_HIGH_SPEED         885
#define DFT_TAIL_COMMAND            PARAM_TAIL_ST
#define DFT_TAIL_POSITION           180.0
#define DFT_TAIL_WAG_PERIODMS       300

#endif