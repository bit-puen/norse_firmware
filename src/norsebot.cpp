#include "norsebot.h"


// NorseBot::NorseBot(dynamixel_config_t *configMotor, protocol_config_t *configProtocol)
NorseBot::NorseBot(HardwareSerial& commandPort, HardwareSerial& dynamixelPort, uint8_t directionPin, uint8_t obstaclePin)
    : _commandPort(commandPort), _dynamixelPort(dynamixelPort), _obstaclePin(obstaclePin)
{
    _motor = new Dynamixel2Arduino(_dynamixelPort, directionPin);
    _protocol = new NorseProtocol(_commandPort, 115200);
    _buildinLed = new Adafruit_NeoPixel(1, GPIO_NUM_38, NEO_GRB + NEO_KHZ800);
    pinMode(_obstaclePin, INPUT);
} 

NorseBot::~NorseBot()
{
}

void NorseBot::init()
{
    _motor->begin(115200);
    _motor->setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
    _protocol->begin();
    _buildinLed->begin();
    _buildinLed->clear();

    readingThreadRunning = true;
    xTaskCreatePinnedToCore(this->protocolThread, "protocolTask", 2048, this, 1, &taskProtocol, 1);
    
    _norsebotConfig.lengthWheelToCenterX = 0.119;
    _norsebotConfig.lengthWheelToCenterY = 0.119;
    _norsebotConfig.wheelRadius = 0.03;

    _targetPositionX = 0.0;
    _targetPositionY = 0.0;
    _targetPhi = 0.0;
    _periodS = 0.1;

    reset();
}

void NorseBot::protocolHandler()
{
    float tarPosX, tarPosY, tarPhi;
    switch (_rxPacket.eventId)
    {
        case EVENT_DRIVING_MODE:
            _norsebotStatus.controlMode = _rxPacket.parameters[0];
            break;
        case EVENT_DRIVING_MANUAL: 
            if (_norsebotStatus.controlMode != DRIVING_MODE_MANUAL) 
            {
                _protocol->respondError(ERR_PERMISSION); 
            }
            else
            {
                _norsebotStatus.currentManualCommand = _rxPacket.parameters[0];
                _norsebotStatus.currentManualSpeed = BYTES2UINT16(_rxPacket.parameters[2], _rxPacket.parameters[1]);
                // _norsebotStatus.currentManualSpeed = (((uint16_t)_rxPacket.parameters[2]) << 8) | (uint16_t)_rxPacket.parameters[1];
            }
            break;
        case EVENT_DRIVING_AUTO:
            if (_norsebotStatus.controlMode != DRIVING_MODE_AUTO) 
            {
                _protocol->respondError(ERR_PERMISSION); 
            }
            else
            {
                tarPosX = float(BYTES2UINT16(_rxPacket.parameters[1], _rxPacket.parameters[0])) / 1000;
                tarPosY = float(BYTES2UINT16(_rxPacket.parameters[3], _rxPacket.parameters[2])) / 1000;
                _targetPhi = DEG2RAD(float(BYTES2INT16(_rxPacket.parameters[5], _rxPacket.parameters[4])));
                // RotZ(-90) robot frame x-axis along world frame y-axis
                if (_targetPhi > 0) tarPhi = (-1) * _targetPhi;
                else                tarPhi = _targetPhi;
                
                _targetPositionX = (tarPosX * cos(tarPhi)) - (tarPosY * sin(tarPhi));
                _targetPositionY = (tarPosX * sin(tarPhi)) + (tarPosY * cos(tarPhi));
            }

            // ESP_LOGV(TAG_PROTOCOL, "TarX: %.2f \t TarY: %.2f \t TarPhi: %.2f", _targetPositionX, _targetPositionY, _targetPhi);
        case EVENT_DRIVING_OVERRIDE:
            if (_norsebotStatus.controlMode != DRIVING_MODE_OVERRIDE) 
            {
                _protocol->respondError(ERR_PERMISSION); 
            }
            else
            {
                _targetOmegaFR = BYTES2INT16(_rxPacket.parameters[1], _rxPacket.parameters[0]);
                _targetOmegaFL = BYTES2INT16(_rxPacket.parameters[3], _rxPacket.parameters[2]);
                _targetOmegaRL = BYTES2INT16(_rxPacket.parameters[5], _rxPacket.parameters[4]);
                _targetOmegaRR = BYTES2INT16(_rxPacket.parameters[7], _rxPacket.parameters[6]);
            }
        default: break;
    }
}

void NorseBot::updateControl()
{
    switch (_norsebotStatus.controlMode)
    {
        case DRIVING_MODE_MANUAL:   manualDriveHandler(); break;
        case DRIVING_MODE_AUTO:     autoDriveHandler(); break;
        case DRIVING_MODE_OVERRIDE: overrideDriveHandler(); break;
        default: break;
    }
}

void NorseBot::manualDriveHandler()
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
        case PARAM_MOVING_AB_CW: moveAroundBendCw(_norsebotStatus.currentManualSpeed); break;
        case PARAM_MOVING_AB_CCW: moveAroundBendCcw(_norsebotStatus.currentManualSpeed); break;
        default: break;
    }
    _norsebotStatus.currentManualCommand = 0xFF;
}

void NorseBot::autoDriveHandler()
{
    float positioningVelocity = 0.035;  // rad/s
    float maxVelocity = 0.079;  // m/s
    float trayNearTarget = 0.01;  // m/s
    // float expectedVelocityX, expectedVelocityY, expectedOmegaZ;
    float distanceX, distanceY, alpha;

    distanceX = _targetPositionX - _odometryPositionX;
    distanceY = _targetPositionY - _odometryPositionY;
    // ESP_LOGV(TAG_PROTOCOL, "TarX: %.2f \t TarY: %.2f \t TarPhi: %.2f", _targetPositionX, _targetPositionY, _targetPositionPhi);
    // printf("distanceX: %.2f || distanceY: %.2f \r\n", distanceX, distanceY);

    // Calculate velocity on velocity profile
    velocityProfileSquare(positioningVelocity, maxVelocity, trayNearTarget, distanceX, distanceY);
    // printf("expectedVelocityX: %.2f || expectedVelocityY: %.2f || ", expectedVelocityX, expectedVelocityY);
    alpha = atan2(distanceY, distanceX);
    // printf("alpha: %.2f\r\n", alpha);
    // expectedPhi = 0;  // fix orientation;
    expectedPhi = 0;
    expectedVelocityX = expectedVelocityX * cos(alpha);
    expectedVelocityY = expectedVelocityY * sin(alpha);

    /* PID Orientation */
    if (true)
    {
        float maxUk = 1.5;
        float minUk = -1.5;
        float error = _targetPhi - _odometryPhi;
        float Ik = error * _periodS + _IkOrientationOld;

        if (Ik > maxUk)             Ik = maxUk;
        else if (Ik < minUk)        Ik = minUk;

        float Uk = error * 0.5 + Ik * 0.02;

        if (Uk < minUk)             Uk = minUk;
        else if (Uk > maxUk)        Uk = maxUk;
        if (abs(error) < 0.005)     Uk = 0;

        _IkOrientationOld = Ik;
        expectedPhi = Uk;
    }

    // ESP_LOGI(TAG_AUTODRIVE, "expectedVelocityX: %.2f || expectedVelocityY: %.2f || expectedPhi: %.2f", expectedVelocityX, expectedVelocityY, expectedPhi);
    
    /* Inverse kinematics */
    _expectedOmegaFR = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX + expectedVelocityY + (expectedPhi * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));
    _expectedOmegaFL = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX - expectedVelocityY - (expectedPhi * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));
    _expectedOmegaRL = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX + expectedVelocityY - (expectedPhi * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));
    _expectedOmegaRR = (1 / _norsebotConfig.wheelRadius) * (expectedVelocityX - expectedVelocityY + (expectedPhi * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));

    _expectedOmegaFR = RADPERSEC2RPM(_expectedOmegaFR);
    _expectedOmegaFL = RADPERSEC2RPM(_expectedOmegaFL);
    _expectedOmegaRL = RADPERSEC2RPM(_expectedOmegaRL);
    _expectedOmegaRR = RADPERSEC2RPM(_expectedOmegaRR);

    // ESP_LOGI(TAG_AUTODRIVE, "omegaFR: %.2f \t omegaFL: %.2f \t omegaRL: %.2f \t omegaRR: %.2f", _expectedOmegaFR, _expectedOmegaFL, _expectedOmegaRL, _expectedOmegaRR);

    if (_expectedOmegaFR > MOTOR_MAX_RPM)                _expectedOmegaFR = MOTOR_MAX_RPM;
    else if (_expectedOmegaFR < (-1) * MOTOR_MAX_RPM)    _expectedOmegaFR = (-1) * MOTOR_MAX_RPM;
    if (_expectedOmegaFL > MOTOR_MAX_RPM)                _expectedOmegaFL = MOTOR_MAX_RPM;
    else if (_expectedOmegaFL < (-1) * MOTOR_MAX_RPM)    _expectedOmegaFL = (-1) * MOTOR_MAX_RPM;
    if (_expectedOmegaRL > MOTOR_MAX_RPM)                _expectedOmegaRL = MOTOR_MAX_RPM;
    else if (_expectedOmegaRL < (-1) * MOTOR_MAX_RPM)    _expectedOmegaRL = (-1) * MOTOR_MAX_RPM;
    if (_expectedOmegaRR > MOTOR_MAX_RPM)                _expectedOmegaRR = MOTOR_MAX_RPM;
    else if (_expectedOmegaRR < (-1) * MOTOR_MAX_RPM)    _expectedOmegaRR = (-1) * MOTOR_MAX_RPM;

    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, (int)_expectedOmegaFR, UNIT_RPM);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, (int)_expectedOmegaFL, UNIT_RPM);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, (int)_expectedOmegaRL, UNIT_RPM);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID,(int)_expectedOmegaRR, UNIT_RPM);
}

void NorseBot::overrideDriveHandler()
{
    if (_motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, _targetOmegaFR, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, _targetOmegaFL, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, _targetOmegaRL, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, _targetOmegaRR, UNIT_RAW))
    {
        _protocol->respondOk(EVENT_DRIVING_OVERRIDE);
    }
}

void NorseBot::updatePosition()
{
    float presentPositionFL, presentPositionFR, presentPositionRL, presentPositionRR;

    _norsebotStatus.presentPositionFL = _motor->getPresentPosition(WHEEL_FRONT_LEFT_ID, UNIT_DEGREE);
    _norsebotStatus.presentPositionFR = _motor->getPresentPosition(WHEEL_FRONT_RIGHT_ID, UNIT_DEGREE);
    _norsebotStatus.presentPositionRL = _motor->getPresentPosition(WHEEL_REAR_LEFT_ID, UNIT_DEGREE);
    _norsebotStatus.presentPositionRR = _motor->getPresentPosition(WHEEL_REAR_RIGHT_ID, UNIT_DEGREE);

    // // ESP_LOGI(TAG_DYNAMIXEL, "Present motor position:\t %.2f || %.2f || %.2f || %.2f",   _norsebotStatus.presentPositionFR,
    // //                                                                                     _norsebotStatus.presentPositionFL,
    // //                                                                                     _norsebotStatus.presentPositionRL,
    // //                                                                                     _norsebotStatus.presentPositionRR);

    presentPositionFR = _norsebotStatus.presentPositionFR - _norsebotStatus.initialPositionFR;
    presentPositionFL = _norsebotStatus.presentPositionFL - _norsebotStatus.initialPositionFL;
    presentPositionRL = _norsebotStatus.presentPositionRL - _norsebotStatus.initialPositionRL;
    presentPositionRR = _norsebotStatus.presentPositionRR - _norsebotStatus.initialPositionRR;

    // ESP_LOGI(TAG_DYNAMIXEL, "Present motor position:\t %.2f || %.2f || %.2f || %.2f",   presentPositionFR,
    //                                                                                     presentPositionFL,
    //                                                                                     presentPositionRL,
    //                                                                                     presentPositionRR);

    /* Forward kinematics */
    _odometryPositionX = (DEG2RAD(presentPositionFR) + 
                          DEG2RAD(presentPositionFL) +
                          DEG2RAD(presentPositionRL) + 
                          DEG2RAD(presentPositionRR)) * 
                          (_norsebotConfig.wheelRadius / 4);
    _odometryPositionY = (DEG2RAD(presentPositionFR) - 
                          DEG2RAD(presentPositionFL) +
                          DEG2RAD(presentPositionRL) - 
                          DEG2RAD(presentPositionRR)) * 
                          (_norsebotConfig.wheelRadius / 4);
    _odometryPhi =       (DEG2RAD(presentPositionFR) - 
                          DEG2RAD(presentPositionFL) -
                          DEG2RAD(presentPositionRL) + 
                          DEG2RAD(presentPositionRR)) * 
                          (_norsebotConfig.wheelRadius / (4 * (_norsebotConfig.lengthWheelToCenterX + _norsebotConfig.lengthWheelToCenterY)));

    // float fwkVelocityX, fwkVelocityY, odometryPhi;
    // float omegaFR, omegaFL, omegaRL, omegaRR;
    // omegaFR = RPM2RADPERSEC(_motor->getPresentVelocity(WHEEL_FRONT_RIGHT_ID, UNIT_RPM));
    // omegaFL = RPM2RADPERSEC(_motor->getPresentVelocity(WHEEL_FRONT_LEFT_ID, UNIT_RPM));
    // omegaRL = RPM2RADPERSEC(_motor->getPresentVelocity(WHEEL_REAR_LEFT_ID, UNIT_RPM));
    // omegaRR = RPM2RADPERSEC(_motor->getPresentVelocity(WHEEL_REAR_RIGHT_ID, UNIT_RPM));

    // /* Forward kinematics */
    // _fwkVelocityX = (_norsebotConfig.wheelRadius / 4) * (omegaFR + omegaFL + omegaRL + omegaRR);
    // _fwkVelocityY = (_norsebotConfig.wheelRadius / 4) * (omegaFR - omegaFL + omegaRL - omegaRR);
    // // // RotZ(90) bc 0 along x-axis
    // _odometryPositionX = _odometryPositionX + (_fwkVelocityX * _periodS);
    // _odometryPositionY = _odometryPositionY + (_fwkVelocityY * _periodS); 
    // _odometryPhi = _odometryPhi + (_fwkPhi * _periodS);
    // if (abs(_odometryPhi) >= 3.14)
    // {
    //     _odometryPhi = (-1) * _odometryPhi;
    // }
    
    ESP_LOGV(TAG_AUTODRIVE, "TarX: %.2f || TarY: %.2f || TarPhi: %.2f", _targetPositionX, _targetPositionY, _targetPhi);
    ESP_LOGV(TAG_AUTODRIVE, "OdoX: %.2f || OdoY: %.2f || OdoPhi: %.2f", _odometryPositionX, _odometryPositionY, _odometryPhi);
}

void NorseBot::updateObstacle()
{
    uint8_t detection = 0;
    detection = digitalRead(_obstaclePin);
    if (detection)
    {
        // digitalWrite(PIN_NEOPIXEL, HIGH);
        _buildinLed->setPixelColor(0, _buildinLed->Color(255, 0, 0));
        _protocol->respondError(ERR_OBSTABLE);
        ESP_LOGI(TAG_OBSTACLE, "Obstacle was detected!");
    }
    else
    {
        _buildinLed->setPixelColor(0, _buildinLed->Color(0, 0, 0));
    }
    _buildinLed->show();
}

void NorseBot::reset()
{
    initNorsebotStatus();
    startEngine();
}

void NorseBot::initNorsebotStatus()
{
    // ping
    // printf("Ping motor... \r\n");
    for (size_t i = 1; i <= 4; i++)
    {
        while (true)
        {
            if (_motor->ping(i))
            {
                ESP_LOGI(TAG_DYNAMIXEL, "ID %d: ping succeeded", i);
                break;
            }
        }
    }
    
    _motor->reboot(WHEEL_FRONT_RIGHT_ID);
    _motor->reboot(WHEEL_FRONT_LEFT_ID);
    _motor->reboot(WHEEL_REAR_LEFT_ID);
    _motor->reboot(WHEEL_REAR_RIGHT_ID);
    delay(100);
    
    _norsebotStatus.initialPositionFL = _motor->getPresentPosition(WHEEL_FRONT_LEFT_ID, UNIT_DEGREE);
    _norsebotStatus.initialPositionFR = _motor->getPresentPosition(WHEEL_FRONT_RIGHT_ID, UNIT_DEGREE);
    _norsebotStatus.initialPositionRL = _motor->getPresentPosition(WHEEL_REAR_LEFT_ID, UNIT_DEGREE);
    _norsebotStatus.initialPositionRR = _motor->getPresentPosition(WHEEL_REAR_RIGHT_ID, UNIT_DEGREE);
    _norsebotStatus.currentManualSpeed = 0;
    _norsebotStatus.controlMode = DRIVING_MODE_MANUAL;
    _norsebotStatus.currentManualCommand = PARAM_MOVING_ST;

    _norsebotStatus.presentPositionFL = 0;
    _norsebotStatus.presentPositionFR = 0;
    _norsebotStatus.presentPositionRL = 0;
    _norsebotStatus.presentPositionRR = 0;

    _odometryPositionX = 0; 
    _odometryPositionY = 0;
    _odometryPhi = 0;
    ESP_LOGI(TAG_DYNAMIXEL, "Initial motor position:\t %.2f\t%.2f\t%.2f\t%.2f\t",   _norsebotStatus.initialPositionFR,
                                                                            _norsebotStatus.initialPositionFL,
                                                                            _norsebotStatus.initialPositionRL,
                                                                            _norsebotStatus.initialPositionRR);
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
    if (_motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, UNIT_RAW))
    {
        _protocol->respondOk(EVENT_DRIVING_MANUAL);
    }
}

void NorseBot::moveForward(uint16_t speed)
{
    if (_motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, UNIT_RAW))
    {
        _protocol->respondOk(EVENT_DRIVING_MANUAL);
    }

}

void NorseBot::moveBackward(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, -speed, UNIT_RAW);
}

void NorseBot::moveStraightLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, -speed, UNIT_RAW);
}

void NorseBot::moveStraightRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, UNIT_RAW);
}

void NorseBot::moveForwardLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, UNIT_RAW);
}

void NorseBot::moveForwardRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, UNIT_RAW);
}

void NorseBot::moveBackwardLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, -speed, UNIT_RAW);
}

void NorseBot::moveBackwardRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, UNIT_RAW);
}

void NorseBot::moveRotateLeft(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, UNIT_RAW);
}

void NorseBot::moveRotateRight(uint16_t speed)
{
    _protocol->respondOk(EVENT_DRIVING_MANUAL);
    _motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, -speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, UNIT_RAW);
    _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, -speed, UNIT_RAW);
}

void NorseBot::moveAroundBendCw(uint16_t speed)
{
    if (_motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, 0, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, speed, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, speed, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, 0, UNIT_RAW))
    {
        _protocol->respondOk(EVENT_DRIVING_MANUAL);
    }
}

void NorseBot::moveAroundBendCcw(uint16_t speed)
{
    if (_motor->setGoalVelocity(WHEEL_FRONT_RIGHT_ID, speed, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_FRONT_LEFT_ID, 0, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_LEFT_ID, 0, UNIT_RAW) &&
        _motor->setGoalVelocity(WHEEL_REAR_RIGHT_ID, speed, UNIT_RAW))
    {
        _protocol->respondOk(EVENT_DRIVING_MANUAL);
    }
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

    if (trayDistanceX <= targetZone)            expectedVelocityX = 0;
    else if (trayDistanceX < nearTargetZone)    expectedVelocityX = positioningVelocity;
    else                                        expectedVelocityX = trayMaxVelocity;

    if (trayDistanceY <= targetZone)            expectedVelocityY = 0; 
    else if (trayDistanceY < nearTargetZone)    expectedVelocityY = positioningVelocity;
    else                                        expectedVelocityY = trayMaxVelocity;
    
    // return expectVelocityX, expectVelocityY;
}

void NorseBot::protocolThread(void *pvParamter)
{
    NorseBot *instance = (NorseBot*)pvParamter;
    instance->protocolThreadWorker();
}

void NorseBot::protocolThreadWorker()
{
    ESP_LOGI(TAG_PROTOCOL, "initiate protocol thread");
    while (readingThreadRunning)
    {
        _protocol->runCommunication();
        isPacketAvilable = _protocol->getIsPacketAvilable();
        if (isPacketAvilable)
        {
            _rxPacket = _protocol->getPacket();
            protocolHandler();
        }
    }
}

