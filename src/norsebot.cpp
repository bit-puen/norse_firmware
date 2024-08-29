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
    // printf("Engine start \r\n");
    // wait_us(500);
    readingThreadRunning = true;
    _protocolThread->start(mbed::callback(protocolThread, this));

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
        case NBOT_REG_CMDMODE_MANUAL: manualModeHandler(); break;
        case NBOT_REG_CMDMODE_AUTO: autoModeHandler(); break;
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

void NorseBot::autoModeHandler()
{
    float positioningVelocity = 0.035;  // rad/s
    float maxVelocity = 6.283185;  // rad/s
    float trayNearTarget = 0.01;  // rad/s
    // float expectedVelocityX, expectedVelocityY, expectedOmegaZ;
    float distanceX, distanceY, alpha;
    float omegaFL, omegaFR, omegaRL, omegaRR;

    distanceX = _targetPositionX - _odometryPositionX;
    distanceY = _targetPositionY - _odometryPositionY;
    printf("distanceX: %d || distanceY: %d \r\n", int(distanceX*100), int(distanceY*100));

    // Calculate velocity on velocity profile
    // expectedVelocityX, expectedVelocityY = velocityProfileSquare(positioningVelocity, maxVelocity, trayNearTarget, distanceX, distanceY);
    velocityProfileSquare(positioningVelocity, maxVelocity, trayNearTarget, distanceX, distanceY);
    printf("expectedVelocityX: %d || expectedVelocityY: %d || ", int(expectedVelocityX*100), int(expectedVelocityY*100));
    alpha = atan2(distanceY, distanceX);
    printf("alpha: %d\r\n", int(alpha*100));
    expectedOmegaZ = 0;  // fix orientation;
    expectedVelocityX = expectedVelocityX * cos(alpha);
    expectedVelocityY = expectedVelocityY * sin(alpha);

    // Check limit of motor
    if (expectedVelocityX > maxVelocity)
    {
        expectedVelocityX = maxVelocity;
    }
    else if (expectedVelocityX < (-1) * maxVelocity)
    {
       expectedVelocityX = (-1) * maxVelocity;
    }
    
    if (expectedVelocityY > maxVelocity)
    {
        expectedVelocityY = maxVelocity;
    }
    else if (expectedVelocityY < (-1) * maxVelocity)
    {
       expectedVelocityY = (-1) * maxVelocity;
    }

    printf("expectedVelocityX: %d || expectedVelocityY: %d \r\n", int(expectedVelocityX*100), int(expectedVelocityY*100));
    
    /* Inverse kinematics */
    omegaFL = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX - expectedVelocityY - (expectedOmegaZ * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));
    omegaFR = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX + expectedVelocityY + (expectedOmegaZ * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));
    omegaRL = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX + expectedVelocityY - (expectedOmegaZ * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));
    omegaRR = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX - expectedVelocityY + (expectedOmegaZ * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));

    omegaFL = RPM2LSB(RADPERSEC2RPM(omegaFL));
    omegaFR = RPM2LSB(RADPERSEC2RPM(omegaFR));
    omegaRL = RPM2LSB(RADPERSEC2RPM(omegaRL));
    omegaRR = RPM2LSB(RADPERSEC2RPM(omegaRR));

    // printf("omegaFL: %d \t omegaFR: %d \t omegaRL: %d \t omegaRR: %d\r\n", (int)omegaFL, (int)omegaFR, (int)omegaRL, (int)omegaRR);

    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, (int)omegaFR);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, (int)omegaFL);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, (int)omegaRL);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID,(int)omegaRR);
}

void NorseBot::updateMotorPosition()
{
    int motorPositionFL, motorPositionFR, motorPositionRL, motorPositionRR;
    int presentPositionFL, presentPositionFR, presentPositionRL, presentPositionRR;
    
    motorPositionFL = _motor->getPresentPosition(WHEEL_FRONT_LEFT_ID);
    if (motorPositionFL != 0)
    {
        _norsebotStatus.presentPositionFL = motorPositionFL;
    }
    motorPositionFR = _motor->getPresentPosition(WHEEL_FRONT_RIGHT_ID);
    if (motorPositionFR != 0)
    {
        _norsebotStatus.presentPositionFR = motorPositionFR;
    }
    motorPositionRL = _motor->getPresentPosition(WHEEL_REAR_LEFT_ID);
    if (motorPositionRL != 0)
    {
        _norsebotStatus.presentPositionRL = motorPositionRL;
    }
    motorPositionRR = _motor->getPresentPosition(WHEEL_REAR_RIGHT_ID);
    if (motorPositionRR != 0)
    {
        _norsebotStatus.presentPositionRR = motorPositionRR;
    }

    /* Forward kinematics */
    presentPositionFL = _norsebotStatus.presentPositionFL - _norsebotStatus.initialPositionFL;
    presentPositionFR = _norsebotStatus.presentPositionFR - _norsebotStatus.initialPositionFR;
    presentPositionRL = _norsebotStatus.presentPositionRL - _norsebotStatus.initialPositionRL;
    presentPositionRR = _norsebotStatus.presentPositionRR - _norsebotStatus.initialPositionRR;

    _odometryPositionX = (DEG2RAD(LSB2DEGREE(presentPositionFL)) + 
                          DEG2RAD(LSB2DEGREE(presentPositionFR)) +
                          DEG2RAD(LSB2DEGREE(presentPositionRL)) + 
                          DEG2RAD(LSB2DEGREE(presentPositionRR))) * 
                          (_norsebotConfig.wheelRadius / 4);
    _odometryPositionY = (-DEG2RAD(LSB2DEGREE(presentPositionFL)) + 
                          DEG2RAD(LSB2DEGREE(presentPositionFR)) +
                          DEG2RAD(LSB2DEGREE(presentPositionRL)) - 
                          DEG2RAD(LSB2DEGREE(presentPositionRR))) * 
                          (_norsebotConfig.wheelRadius / 4);
    _odometryPositionTheta = (-DEG2RAD(LSB2DEGREE(presentPositionFL)) + 
                          DEG2RAD(LSB2DEGREE(presentPositionFR)) -
                          DEG2RAD(LSB2DEGREE(presentPositionRL)) + 
                          DEG2RAD(LSB2DEGREE(presentPositionRR))) * 
                          (_norsebotConfig.wheelRadius / (4 * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));
    
    printf("PositionFL: %d \r\n", presentPositionFL);
    printf("PositionFR: %d \r\n", int(LSB2DEGREE(presentPositionFL)));
    printf("PositionRL: %d \r\n", int(DEG2RAD(LSB2DEGREE(presentPositionFL))));
    printf("PositionRR: %d \r\n", presentPositionRR);

    printf("_odometryPositionX: %d || _odometryPositionY: %d || _odometryPositionTheta: %d\r\n", int(_odometryPositionX*100), int(_odometryPositionY*100), int(_odometryPositionTheta*100));

    // _norsebotStatus.presentPositionFL = _motor->getPresentPosition(WHEEL_FRONT_LEFT_ID)  - _norsebotStatus.initialPositionFL;
    // _norsebotStatus.presentPositionFR = _motor->getPresentPosition(WHEEL_FRONT_RIGHT_ID) - _norsebotStatus.initialPositionFR;
    // _norsebotStatus.presentPositionRL = _motor->getPresentPosition(WHEEL_REAR_LEFT_ID)   - _norsebotStatus.initialPositionRL;
    // _norsebotStatus.presentPositionRR = _motor->getPresentPosition(WHEEL_REAR_RIGHT_ID)  - _norsebotStatus.initialPositionRR;
   
    // printf("presentPositionFL: %d\r\n", _norsebotStatus.presentPositionFL);
    // printf("presentPositionFR: %d\r\n", _norsebotStatus.presentPositionFR);
    // printf("presentPositionRL: %d\r\n", _norsebotStatus.presentPositionRL);
    // printf("presentPositionRR: %d\r\n", _norsebotStatus.presentPositionRR);


}

void NorseBot::reset()
{
    initNorsebotStatus();
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

void NorseBot::initNorsebotStatus()
{
    _norsebotStatus.initialPositionFL = _motor->getPresentPosition(WHEEL_FRONT_LEFT_ID);
    _norsebotStatus.initialPositionFR = _motor->getPresentPosition(WHEEL_FRONT_RIGHT_ID);
    _norsebotStatus.initialPositionRL = _motor->getPresentPosition(WHEEL_REAR_LEFT_ID);
    _norsebotStatus.initialPositionRR = _motor->getPresentPosition(WHEEL_REAR_RIGHT_ID);
    _norsebotStatus.currentManualSpeed = 0;
    _norsebotStatus.controlMode = CONTROL_MODE_MANUAL;
    _norsebotStatus.currentManualCommand = PARAM_MOVING_ST;

    _norsebotStatus.presentPositionFL = 0;
    _norsebotStatus.presentPositionFR = 0;
    _norsebotStatus.presentPositionRL = 0;
    _norsebotStatus.presentPositionRR = 0;

    _odometryPositionX = 0; 
    _odometryPositionY = 0;
    _odometryPositionTheta = 0;
    printf("initialPositionFL: %d\r\n", _norsebotStatus.initialPositionFL);
    printf("initialPositionFR: %d\r\n", _norsebotStatus.initialPositionFR);
    printf("initialPositionRL: %d\r\n", _norsebotStatus.initialPositionRL);
    printf("initialPositionRR: %d\r\n", _norsebotStatus.initialPositionRR);
}

void NorseBot::velocityProfileSquare(float positioningVelocity, float trayMaxVelocity, float targetZone, float distanceX, float distanceY)
{
    float trayDistanceX, trayDistanceY;
    // float expectVelocityX, expectVelocityY;
    float nearTargetZone; 
    
    nearTargetZone = 10 * targetZone;

    // Calculate distance and velocity to target
    trayDistanceX = abs(distanceX);
    trayDistanceY = abs(distanceY);

    if (trayDistanceX <= targetZone)
    {
        expectedVelocityX = 0; 
    }
    else if (trayDistanceX < nearTargetZone)
    {
        expectedVelocityX = positioningVelocity;
    }
    else
    {
        expectedVelocityX = trayMaxVelocity;
    }

    if (trayDistanceY <= targetZone)
    {
        expectedVelocityY = 0; 
    }
    else if (trayDistanceY < nearTargetZone)
    {
        expectedVelocityY = positioningVelocity;
    }
    else
    {
        expectedVelocityY = trayMaxVelocity;
    }
    
    // return expectVelocityX, expectVelocityY;
}

void NorseBot::protocolThread(void const *pvParamter)
{
    NorseBot *instance = (NorseBot*)pvParamter;
    instance->protocolThreadWorker();
}

void NorseBot::protocolThreadWorker()
{
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
