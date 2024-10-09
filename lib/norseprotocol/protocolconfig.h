#ifndef PROTOCOL_CONFIG_H
#define PROTOCOL_CONFIG_H

#define NUMBER_HEADER           2
#define MAXIMUM_BUFFER_SIZE     12
#define START_BYTE_1            0xFF
#define START_BYTE_2            0xFA
#define STOP_BYTE_1             0x0D
#define STOP_BYTE_2             0x0A
/* Event */
#define EVENT_PING              0x90
#define EVENT_REBOOT            0x91
#if PTC_MAJOR_VER == 1
#define EVENT_DRIVING_MANUAL    0xA1
#elif PTC_MAJOR_VER >= 2
#define EVENT_DRIVING_MODE      0xA0
#define EVENT_DRIVING_MANUAL    0xA1
#define EVENT_DRIVING_AUTO      0xA2
#define EVENT_DRIVING_OVERRIDE  0xA3
#define EVENT_TAIL_MODE         0xB0
#define EVENT_TAIL_CMD          0xB1
#endif
#define EVENT_REQUEST           0xF0
#define EVENT_RESPONSE_OK       0x00
#define EVENT_RESPONSE_DATA     0x01
#define EVENT_RESPONSE_ERROR    0xFF

/* PARAMS */
// Ping
#define PARAM_PING              0xA0
// Reboot
#define PARAM_REBOOT_MOTOR      0x00
#define PARAM_REBOOT_STAT       0x01
#define PARAM_REBOOT_NBOT       0x02
// Driving mode
#define PARAM_MANUAL_DRIVE_MODE 0x00
#define PARAM_AUTO_DRIVE_MODE   0x01
#define PARAM_OVERRIDE_MODE     0x02
// Manual drive mode
#define PARAM_MOVING_ST         0x00
#define PARAM_MOVING_FW         0x01
#define PARAM_MOVING_BW         0x02
#define PARAM_MOVING_SL         0x03
#define PARAM_MOVING_SR         0X04
#define PARAM_MOVING_RL         0X05
#define PARAM_MOVING_RR         0X06
#define PARAM_MOVING_FL         0X07
#define PARAM_MOVING_FR         0X08
#define PARAM_MOVING_BL         0X09
#define PARAM_MOVING_BR         0X0A
#define PARAM_MOVING_AB_CW      0X0B
#define PARAM_MOVING_AB_CCW     0X0C
// Tail mode
#define PARAM_TAIL_POS_CONTROL  0x00
#define PARAM_TAIL_POT_CONTROL  0x01
// Tail manual mode
#define PARAM_TAIL_ST           0x00
#define PARAM_TAIL_LS           0x01
#define PARAM_TAIL_HS           0x02
/* Error reason code */
#define ERR_HEADER              0x00
#define ERR_CHECKSUM            0x01
#define ERR_PERMISSION          0x02
#if PTC_MAJOR_VER >= 2 && PTC_MINOR_VER >=2
#define ERR_OBSTABLE            0x03
#define ERR_MOTOR_CMD_FAIL      0x04
#endif



#endif