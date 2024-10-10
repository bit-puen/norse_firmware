#ifndef NORSEBOT_H
#define NORSEBOT_H

// #include <esp_log.h>
#include "norseutils.h"
#include "config/pin.h"
#include "config/constant .h"
#include "norseprotocol.h"
#include "Dynamixel2Arduino.h"
#include "inahandler.h"
#include <Adafruit_NeoPixel.h>

#define TAG_PROTOCOL                "Protocol"
#define TAG_DYNAMIXEL               "Dynamixel"
#define TAG_AUTODRIVE               "AutoDrive"
#define TAG_OBSTACLE                "Obstacle"
#define TAG_BATTERY                 "Battery"

typedef struct
{
    float lX;
    float lY;
    float r;
} norsebot_config_t;

typedef struct
{
    float initialPositionFL;
    float initialPositionFR;
    float initialPositionRL;
    float initialPositionRR;
    float presentPositionFL;
    float presentPositionFR;
    float presentPositionRL;
    float presentPositionRR;
    uint8_t controlMode;
    uint16_t manualSpeed;
    uint16_t manualCommand;
} norsebot_status_t;

typedef struct
{
    uint8_t mode;
    uint16_t range;
    uint16_t lowSpeed;
    uint16_t highSpeed;
    uint8_t command;
    uint16_t waggingPeriodMs;
    float position;
} norsebot_tail_status_t;

class NorseBot
{
    public:
        NorseBot(
            HardwareSerial& commandPort, 
            HardwareSerial& dynamixelPort, 
            uint8_t directionPin,
            uint8_t obstaclePin);
        ~NorseBot();

        void init(norsebot_config_t* norsebotConfig);

        void updateBatteryLife();
        void updateControl();
        void updatePosition();
        void updateObstacle();
        void updateTail();

    private:
        //Drive
        void manualDriveHandler();
        void autoDriveHandler();
        void overrideDriveHandler();

        // Tail
        void tailPositionHandler();
        void tailPotHandler();
        
        // Motor
        void startEngine();
        void stopEngine();

        // Other
        void rebootHandler(uint8_t param);
        bool rebootMotor();
        void reboot();
        
        // Norse bot
        void initNorsebotStatus();

        // Norse protocol
        static void protocolThread(void *pvParamter);
        void protocolThreadWorker();
        void protocolHandler();
        void respondDataHandler(uint8_t dataRegister);

    /* Manual drive */
    private:
        void stopMoving();
        void moveForward(uint16_t speed);
        void moveBackward(uint16_t speed);
        void moveStraightLeft(uint16_t speed);
        void moveStraightRight(uint16_t speed);
        void moveForwardLeft(uint16_t speed);
        void moveForwardRight(uint16_t speed);
        void moveBackwardLeft(uint16_t speed);
        void moveBackwardRight(uint16_t speed);
        void moveRotateLeft(uint16_t speed);
        void moveRotateRight(uint16_t speed);
        void moveAroundBendCw(uint16_t speed);
        void moveAroundBendCcw(uint16_t speed);

    /* Auto drive */
    private:
        void velocityProfileSquare(float positioningVelocity, float trayMaxVelocity, float targetZone, float distanceX, float distanceY);

    private:
        NorseProtocol* _protocol;
        HardwareSerial& _commandPort;
        HardwareSerial& _dynamixelPort;
        Dynamixel2Arduino* _motor;
        Adafruit_NeoPixel* _buildinLed;
        TaskHandle_t taskProtocol;
        InaHandler* _ina;

        uint8_t _obstaclePin;
        uint8_t tailState;

        norse_packet_t _rxPacket;
        norsebot_status_t _norsebotStatus;
        norsebot_config_t* _norsebotConfig;
        norsebot_tail_status_t _norsebotTailStatus;

        volatile bool readingThreadRunning = false;
        volatile bool isPacketAvilable = false;
        volatile bool flagDrivingCommand = false;
        
        uint32_t waggingPeriodTimmer = 0;
        int16_t _targetOmegaFR, _targetOmegaFL, _targetOmegaRL, _targetOmegaRR;
        uint16_t batteryVoltage;
    /* Auto drive */
    private:
        // Odometry data
        float _odometryPositionX, _odometryPositionY, _odometryPhi;
        float _fwkVelocityX, _fwkVelocityY, _fwkPhi;
        // Target data
        float _targetPositionX, _targetPositionY, _targetPhi;

        float _expectedOmegaFR, _expectedOmegaFL, _expectedOmegaRL, _expectedOmegaRR;
        float expectedVelocityX, expectedVelocityY, expectedPhi;

        float _IkOrientationOld;
        float _periodS;

};

#endif