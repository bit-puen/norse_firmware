#ifndef PROTOCOL_CONFIG_H
#define PROTOCOL_CONFIG_H

#if PTC_MAJOR_VER == 1
#define NUMBER_HEADER           2
#define MAXIMUM_BUFFER_SIZE     12
#define START_BYTE_1            0xFF
#define START_BYTE_2            0xFA
#define STOP_BYTE_1             0x0D
#define STOP_BYTE_2             0x0A
#define EVENT_PING              0xA0
#define EVENT_MOVING_CMD        0xA1
#define EVENT_REQUEST           0xB0
#define EVENT_RESPONSE_OK       0x00
#define EVENT_RESPONSE_DATA     0x01
#define EVENT_RESPONSE_ERROR    0xFF
/* PARAMS */
#define PARAM_PING              0xA0
/* Error reason code */
#define ERR_HEADER              0x00
#define ERR_CHECKSUM            0x01

#if PTC_MINOR_VER == 0
#define PARAM_MOVING_ST         0x00
#define PARAM_MOVING_FW         0x01
#define PARAM_MOVING_BW         0x02
#define PARAM_MOVING_RL         0x03
#define PARAM_MOVING_RR         0X04
#define PARAM_MOVING_TL         0X05
#define PARAM_MOVING_TR         0X06
#define PARAM_MOVING_FL         0X07
#define PARAM_MOVING_FR         0X08
#define PARAM_MOVING_BL         0X09
#define PARAM_MOVING_BR         0X0A
#endif
#endif

#endif