#ifndef CONFIG_CONSTANT_H
#define CONFIG_CONSTANT_H

#if defined(MK_I) || defined(MK_II)
#define BUADRATE_DYNAMIXEL          1000000
#elif defined(MK_III)
#define BUADRATE_DYNAMIXEL          115200
#define DYNAMIXEL_PROTOCOL_VERSION  2.0
#endif
#define BUADRATE_PROTOCOL           115200

#define MOTOR_MAX_RPM               55

#endif