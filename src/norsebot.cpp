#include "norsebot.h"


// NorseBot::NorseBot(dynamixel_config_t *configMotor, protocol_config_t *configProtocol)
NorseBot::NorseBot(HardwareSerial& commandPort, HardwareSerial& dynamixelPort, uint8_t directionPin)
    : _commandPort(commandPort), _dynamixelPort(dynamixelPort)
{
    _motor = new Dynamixel2Arduino(_dynamixelPort, directionPin);
    _protocol = new NorseProtocol(_commandPort, 115200);

    // _motor = new Dynamixel(configMotor->txPin, configMotor->rxPin, configMotor->baudRate, configMotor->directionPin);
    // _protocol = new NorseProtocol(configProtocol->txPin, configProtocol->rxPin, configProtocol->baudRate);
    // _protocolThread = new Thread(osPriorityNormal, 4096UL, nullptr, "ProtocolThread");
} 

NorseBot::~NorseBot()
{
}

void NorseBot::init()
{
    _motor->begin(115200);
    _protocol->begin();

    startEngine();
    readingThreadRunning = true;
    // _protocolThread->start(mbed::callback(protocolThread, this));
    xTaskCreatePinnedToCore(this->protocolThread, "protocolTask", 16384, this, 1, &task, 1);

    _norsebotConfig.lengthWheelToCenterX = 0.119;
    _norsebotConfig.lengthWheelToCenterY = 0.119;
    _norsebotConfig.wheelRadius = 0.03;

    reset();
    reset();

    _targetPositionX = 0.5;
    _targetPositionY = 0.5;
}

void NorseBot::protocolHandler()
{
    switch (_rxPacket.eventId)
    {
        case EVENT_MOVING_CMD: 
            _norsebotStatus.currentManualCommand = _rxPacket.parameters[0];
            _norsebotStatus.currentManualSpeed = (((uint16_t)_rxPacket.parameters[2]) << 8) | (uint16_t)_rxPacket.parameters[1];
            commandMovingHandler();
            break;
        default: break;
    }
}

void NorseBot::commandMovingHandler()
{
    switch (_norsebotStatus.controlMode)
    {
        case CONTROL_MODE_MANUAL: manualModeHandler(); break;
        case CONTROL_MODE_AUTO: break;
        default: break;
    }
}

void NorseBot::manualModeHandler()
{
    switch (_norsebotStatus.currentManualCommand)
    {
        case PARAM_MOVING_ST: stopMoving(); break;
        case PARAM_MOVING_FW: moveForward(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_BW: moveBackward(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_SL: moveStraightLeft(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_SR: moveStraightRight(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_FL: moveForwardLeft(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_FR: moveForwardRight(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_BL: moveBackwardLeft(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_BR: moveBackwardRight(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_RL: moveRotateLeft(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_RR: moveRotateRight(_norsebotStatus.currentManualSpeed); break;
        default: stopMoving(); break;
    }
}

void NorseBot::reset()
{
}

void NorseBot::startEngine()
{
    _motor->ledOn(WHEEL_FRONT_RIGHT_ID);
    _motor->ledOn(WHEEL_FRONT_LEFT_ID);
    _motor->ledOn(WHEEL_REAR_LEFT_ID);
    _motor->ledOn(WHEEL_REAR_RIGHT_ID);
    _motor->torqueOn(WHEEL_FRONT_RIGHT_ID);
    _motor->torqueOn(WHEEL_FRONT_LEFT_ID);
    _motor->torqueOn(WHEEL_REAR_LEFT_ID);
    _motor->torqueOn(WHEEL_REAR_RIGHT_ID);
}

void NorseBot::stopEngine()
{
    _motor->ledOff(WHEEL_FRONT_RIGHT_ID);
    _motor->ledOff(WHEEL_FRONT_LEFT_ID);
    _motor->ledOff(WHEEL_REAR_LEFT_ID);
    _motor->ledOff(WHEEL_REAR_RIGHT_ID);
    _motor->torqueOff(WHEEL_FRONT_RIGHT_ID);
    _motor->torqueOff(WHEEL_FRONT_LEFT_ID);
    _motor->torqueOff(WHEEL_REAR_LEFT_ID);
    _motor->torqueOff(WHEEL_REAR_RIGHT_ID);
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

void NorseBot::moveStraightLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveStraightRight(uint16_t speed)
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
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveBackwardRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::moveRotateLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_FORWARD);
}

void NorseBot::moveRotateRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_MOVING_CMD);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, MOTOR_DIRECTION_FORWARD);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, MOTOR_DIRECTION_BACKWARD);
}

void NorseBot::protocolThread(void *pvParamter)
{
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
        // vTaskDelay(100);
    }
}

// void NorseBot::protocolThreadWorker()
// {
//     printf("thread\r\n");
//     while (readingThreadRunning)
//     {
//         _protocol->runCommunication();
//         isPacketAvilable = _protocol->getIsPacketAvilable();
//         if (isPacketAvilable)
//         {
//             _rxPacket = _protocol->getPacket();
//             protocolHandler();
//         }
//         // ThisThread::sleep_for(1ms);
//     }
// }
