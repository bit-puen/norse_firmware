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
    startEngine();
    readingThreadRunning = true;
    _protocolThread->start(mbed::callback(protocolThread, this));

    registerControlMode = NBOT_REG_CMDMODE_MANUAL;
    registerModeManualCommand = PARAM_MOVING_ST;
    registerModeManualSpeed = 0;
}

void NorseBot::protocolHandler()
{
    switch (_rxPacket.eventId)
    {
        case EVENT_MOVING_CMD: 
            registerModeManualCommand = _rxPacket.parameters[0];
            registerModeManualSpeed = (((uint16_t)_rxPacket.parameters[2]) << 8) | (uint16_t)_rxPacket.parameters[1];
            commandMovingHandler(); 
            break;
        default: break;
    }
}

void NorseBot::commandMovingHandler()
{
    switch (registerControlMode)
    {
        case NBOT_REG_CMDMODE_MANUAL: manualModeHandler(); break;
        case NBOT_REG_CMDMODE_AUTO: break;
        default: break;
    }
}

void NorseBot::manualModeHandler()
{
    switch (registerModeManualCommand)
    {
        case PARAM_MOVING_ST: stopMoving(); break;
        case PARAM_MOVING_FW: moveForward(registerModeManualSpeed); break;
        case PARAM_MOVING_BW: moveBackward(registerModeManualSpeed); break;
        case PARAM_MOVING_RL: moveRotateLeft(registerModeManualSpeed); break;
        case PARAM_MOVING_RR: moveRotateRight(registerModeManualSpeed); break;
        case PARAM_MOVING_TL: moveForwardLeft(registerModeManualSpeed); break;
        case PARAM_MOVING_TR: moveForwardRight(registerModeManualSpeed); break;
        case PARAM_MOVING_FL: moveBackwardLeft(registerModeManualSpeed); break;
        case PARAM_MOVING_FR: moveBackwardRight(registerModeManualSpeed); break;
        case PARAM_MOVING_BL: moveTurnLeft(registerModeManualSpeed); break;
        case PARAM_MOVING_BR: moveTurnRight(registerModeManualSpeed); break;
        default: stopMoving(); break;
    }
}

void NorseBot::startEngine()
{
    _motor->enableLed(WHEEL_FRONT_RIGHT_ID);
    _motor->enableLed(WHEEL_FRONT_LEFT_ID);
    _motor->enableLed(WHEEL_REAR_LEFT_ID);
    _motor->enableLed(WHEEL_REAR_RIGHT_ID);
    _motor->enableTorque(WHEEL_FRONT_RIGHT_ID);
    _motor->enableTorque(WHEEL_FRONT_LEFT_ID);
    _motor->enableTorque(WHEEL_REAR_LEFT_ID);
    _motor->enableTorque(WHEEL_REAR_RIGHT_ID);
}

void NorseBot::stopEngine()
{
    _motor->disableLed(WHEEL_FRONT_RIGHT_ID);
    _motor->disableLed(WHEEL_FRONT_LEFT_ID);
    _motor->disableLed(WHEEL_REAR_LEFT_ID);
    _motor->disableLed(WHEEL_REAR_RIGHT_ID);
    _motor->disableTorque(WHEEL_FRONT_RIGHT_ID);
    _motor->disableTorque(WHEEL_FRONT_LEFT_ID);
    _motor->disableTorque(WHEEL_REAR_LEFT_ID);
    _motor->disableTorque(WHEEL_REAR_RIGHT_ID);
}

void NorseBot::stopMoving()
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveForward(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveBackward(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveRotateLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveRotateRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveForwardLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveForwardRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveBackwardLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveBackwardRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveTurnLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveTurnRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
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
    printf("thread\r\n");
    while (readingThreadRunning)
    {
        _protocol->runCommunication();
        isPacketAvilable = _protocol->getIsPacketAvilable();
        if (isPacketAvilable)
        {
            _rxPacket = _protocol->getPacket();
            protocolHandler();
        }
        ThisThread::sleep_for(1ms);
    }
}
