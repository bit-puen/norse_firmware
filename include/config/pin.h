#ifndef CONFIG_PIN_H
#define CONFIG_PIN_H

#if defined(MK_I) || defined(MK_II)
#define PIN_DYNAMIXEL_TX            PA_9
#define PIN_DYNAMIXEL_RX            PA_10
#define PIN_DYNAMIXEL_DIRECTION     PA_8
#define PIN_PROTOCOL_TX             USBTX
#define PIN_PROTOCOL_RX             USBRX
#endif

#endif