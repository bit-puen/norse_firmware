#ifndef PROTOCOL_CONFIG_H
#define PROTOCOL_CONFIG_H

#if PTC_MAJOR_VER == 1
#define NUMBER_HEADER           2
#define START_BYTE_1            0xFA
#define START_BYTE_2            0xFF
#define EVENT_PING              0xA0
#define EVENT_MOVING_CMD        0xA1
#define EVENT_REQUEST           0xB0
#define EVENT_RESPONSE_OK       0x00
#define EVENT_RESPONSE_DATA     0x01
#define EVENT_RESPONSE_ERROR    0xFF
/* PARAMS */
#define PARAM_PING              0xA0
#if PTC_MINOR_VER == 0
#define PARAM_MOVING_FW         0x00
#define PARAM_MOVING_BW         0x01
#define PARAM_MOVING_RL         0x02
#define PARAM_MOVING_RR         0X03
#define PARAM_MOVING_TL         0X04
#define PARAM_MOVING_TR         0X05
#define PARAM_MOVING_FL         0X06
#define PARAM_MOVING_FR         0X07
#define PARAM_MOVING_BL         0X08
#define PARAM_MOVING_BR         0X09
#endif
#endif

#endif