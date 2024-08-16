#include "norsebot.h"


NorseBot::NorseBot(dynamixel_config_t *configMotor, protocol_config_t *configProtocol)
{
    _motor = new Dynamixel(configMotor->txPin, configMotor->rxPin, configMotor->baudRate, configMotor->directionPin);
    _protocol = new NorseProtocol(configProtocol->txPin, configProtocol->rxPin, configProtocol->baudRate);
    _protocolThread = new Thread(osPriorityNormal, 4096UL, nullptr, "ProtocolThread");
}

NorseBot::~NorseBot()
{
}

void NorseBot::init()
{
    // NorseProtocol norseProtocol;
    startEngine();
    readingThreadRunning = true;
    // _protocolThread->start(mbed::callback(protocolThread, (void*)""));
    // _protocolThread.start(mbed::callback(protocolThread, (void*)""));
    _protocolThread->start(mbed::callback(protocolThread, this));
}

void NorseBot::startEngine()
{
    _motor->enableTorque(WHEEL_FRONT_RIGHT_ID);
    _motor->enableTorque(WHEEL_FRONT_LEFT_ID);
    _motor->enableTorque(WHEEL_REAR_LEFT_ID);
    _motor->enableTorque(WHEEL_REAR_RIGHT_ID);
}

void NorseBot::stopEngine()
{
    _motor->disableTorque(WHEEL_FRONT_RIGHT_ID);
    _motor->disableTorque(WHEEL_FRONT_LEFT_ID);
    _motor->disableTorque(WHEEL_REAR_LEFT_ID);
    _motor->disableTorque(WHEEL_REAR_RIGHT_ID);
}

void NorseBot::stopMoving()
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveForward(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveBackward(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveLeft(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveRight(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveForwardLeft(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveForwardRight(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveBackwardLeft(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveBackwardRight(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveTurnLeft(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveTurnRight(uint16_t speed)
{
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::protocolThread(void const *pvParamter)
{
    // protocolThreadWorker();
    NorseBot *instance = (NorseBot*)pvParamter;
    instance->protocolThreadWorker();
}

void NorseBot::protocolThreadWorker()
{
    // printf("thread\r\n");
    while (readingThreadRunning)
    {
        _protocol->runCommunication();
        if (_protocol->getIsPacketAvilable())
        {
            _rxPacket = _protocol->getPacket();
        }
        ThisThread::sleep_for(1ms);
    }
}
