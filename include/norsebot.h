#ifndef NORSEBOT_H
#define NORSEBOT_H

#include "config/pin.h"
#include "dynamixel.h"

#ifdef MK_I
#define WHEEL_FRONT_RIGHT_ID        1
#define WHEEL_FRONT_LEFT_ID         2
#define WHEEL_REAR_LEFT_ID          3
#define WHEEL_REAR_RIGHT_ID         4
#endif

#define MOTOR_DIRECTION_FORWARD     0
#define MOTOR_DIRECTION_BACKWARD    1

typedef struct
{
    PinName txPin;
    PinName rxPin;
    PinName directionPin;
    uint32_t baudRate;
} dynamixel_config_t;

typedef struct
{
    float lengthWheelToCenterX;
    float lengthWheelToCenterY;
    float wheelDiameter;
} norsebot_config_t;



class NorseBot
{
    public:
        NorseBot(dynamixel_config_t *config);
        ~NorseBot();

        void startEngine();
        void stopEngine();

        void moveForward(uint16_t speed);
        void moveBackward(uint16_t speed);
        void moveLeft(uint16_t speed);
        void moveRight(uint16_t speed);
        void moveForwardLeft(uint16_t speed);
        void moveForwardRight(uint16_t speed);
        void moveBackwardLeft(uint16_t speed);
        void moveBackwardRight(uint16_t speed);
        void moveTurnLeft(uint16_t speed);
        void moveTurnRight(uint16_t speed);

    private:
        Dynamixel _motor;
};


#endif