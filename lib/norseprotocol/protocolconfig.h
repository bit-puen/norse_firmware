#ifndef PROTOCOL_CONFIG_H
#define PROTOCOL_CONFIG_H

#if PTC_MAJOR_VER == 1
#define NUMBER_HEADER           2
#define START_BYTE_H            0xFA
#define START_BYTE_L            0xFF
#define EVENT_PING              0xA0
#define EVENT_MOVING_CMD        0xA1
#define EVENT_REQUEST           0xB0
#define EVENT_RESPONSE_OK       0x00
#define EVENT_RESPONSE_DATA     0x01
#define EVENT_RESPONSE_ERROR    0xFF
#endif

#endif