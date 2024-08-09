#include "mbed.h"
#include "dynamixel.h"

#define MAXIMUM_BUFFER_SIZE     32


void mecanumKinematics(uint16_t speed, uint16_t strafe, uint16_t turn);
void startEngin();

void stopMoving();
void moveForward(uint16_t velocity);
void moveBackward(uint16_t velocity);
void moveLeft(uint16_t velocity);
void moveRight(uint16_t velocity);
void turnLeft(uint16_t velocity);
void turnRight(uint16_t velocity);
void moveForwardLeftDiagonal(uint16_t velocity);
void moveForwardRightDiagonal(uint16_t velocity);
void moveBackwardLeftDiagonal(uint16_t velocity);
void moveBackwardRightDiagonal(uint16_t velocity);


static BufferedSerial uartUsb(USBTX, USBRX, 115200);

// Dynamixel XL430(USBTX, USBRX, 115200, D7);
Dynamixel XL430(D8, D2, 1000000, D7);


FileHandle *mbed::mbed_override_console(int fd)
{
    return &uartUsb;
}


int main()
{
  uint16_t goalVelocity = 265;
  ThisThread::sleep_for(5s);
  startEngin();

  while (true)
  {
    moveForward(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    moveBackward(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    moveLeft(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    moveRight(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    turnLeft(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    turnRight(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);
  
    moveForwardLeftDiagonal(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    moveBackwardLeftDiagonal(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    moveForwardRightDiagonal(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    moveBackwardRightDiagonal(goalVelocity);
    ThisThread::sleep_for(2s);
    stopMoving();
    ThisThread::sleep_for(2s);

    stopMoving();
    ThisThread::sleep_for(3s);
    wait_us(100);
  }
  return 0;
}

// void mecanumKinematics(uint8_t speed, uint8_t strafe, uint8_t turn)
// {
//   uint16_t frontLeftSpeed = speed + strafe - turn;
//   uint16_t frontRightSpeed = speed - strafe - turn;
//   uint16_t rearLestSpeed = speed - strafe + turn;
//   uint16_t rearRightSpeed = speed + strafe - turn;

//   XL430.setGoalVelocity(1, 255, 0);
// }

void mecanumKinematics(uint16_t speed, uint16_t strafe, uint16_t turn)
{

}

void startEngin()
{
  XL430.enableLed(1);
  XL430.enableLed(2);
  XL430.enableLed(3);
  XL430.enableLed(4);
  XL430.enableTorque(1);
  XL430.enableTorque(2);
  XL430.enableTorque(3);
  XL430.enableTorque(4);
  ThisThread::sleep_for(1s);
  // XL430.disableLed(1);
  // XL430.disableLed(2);
  // XL430.disableLed(3);
  // XL430.disableLed(4);
}

void stopMoving()
{
  printf("stop\r\n");
  XL430.setGoalVelocity(1, 0, 1);
  XL430.setGoalVelocity(2, 0, 1);
  XL430.setGoalVelocity(3, 0, 1);
  XL430.setGoalVelocity(4, 0, 1);
}

void moveForward(uint16_t velocity)
{
  printf("moveForward\r\n");
  XL430.setGoalVelocity(1, velocity, 0);
  XL430.setGoalVelocity(2, velocity, 0);
  XL430.setGoalVelocity(3, velocity, 0);
  XL430.setGoalVelocity(4, velocity, 0);
}

void moveBackward(uint16_t velocity)
{
  printf("moveBackward\r\n");
  XL430.setGoalVelocity(1, velocity, 1);
  XL430.setGoalVelocity(2, velocity, 1);
  XL430.setGoalVelocity(3, velocity, 1);
  XL430.setGoalVelocity(4, velocity, 1);
}

void moveLeft(uint16_t velocity)
{
  printf("moveLeft\r\n");
  XL430.setGoalVelocity(1, velocity, 0);
  XL430.setGoalVelocity(2, velocity, 1);
  XL430.setGoalVelocity(3, velocity, 0);
  XL430.setGoalVelocity(4, velocity, 1);
}

void moveRight(uint16_t velocity)
{
  printf("moveRight\r\n");
  XL430.setGoalVelocity(1, velocity, 1);
  XL430.setGoalVelocity(2, velocity, 0);
  XL430.setGoalVelocity(3, velocity, 1);
  XL430.setGoalVelocity(4, velocity, 0);
}

void turnLeft(uint16_t velocity)
{
  printf("turnLeft\r\n");
  XL430.setGoalVelocity(1, velocity, 0);
  XL430.setGoalVelocity(2, velocity, 1);
  XL430.setGoalVelocity(3, velocity, 1);
  XL430.setGoalVelocity(4, velocity, 0);
}

void turnRight(uint16_t velocity)
{
  printf("turnRight\r\n");
  XL430.setGoalVelocity(1, velocity, 1);
  XL430.setGoalVelocity(2, velocity, 0);
  XL430.setGoalVelocity(3, velocity, 0);
  XL430.setGoalVelocity(4, velocity, 1);
}

void moveForwardLeftDiagonal(uint16_t velocity)
{
  printf("moveForwardLeftDiagonal\r\n");
  XL430.setGoalVelocity(1, velocity, 0);
  XL430.setGoalVelocity(2, 0, 1);
  XL430.setGoalVelocity(3, velocity, 0);
  XL430.setGoalVelocity(4, 0, 1);
}

void moveForwardRightDiagonal(uint16_t velocity)
{
  printf("moveForwardRightDiagonal\r\n");
  XL430.setGoalVelocity(1, 0, 1);
  XL430.setGoalVelocity(2, velocity, 0);
  XL430.setGoalVelocity(3, 0, 1);
  XL430.setGoalVelocity(4, velocity, 0);
}

void moveBackwardLeftDiagonal(uint16_t velocity)
{
  printf("moveBackwardLeftDiagonal\r\n");
  XL430.setGoalVelocity(1, velocity, 1);
  XL430.setGoalVelocity(2, 0, 1);
  XL430.setGoalVelocity(3, velocity, 1);
  XL430.setGoalVelocity(4, 0, 1);
}

void moveBackwardRightDiagonal(uint16_t velocity)
{
  printf("moveBackwardRightDiagonal\r\n");
  XL430.setGoalVelocity(1, 0, 1);
  XL430.setGoalVelocity(2, velocity, 1);
  XL430.setGoalVelocity(3, 0, 1);
  XL430.setGoalVelocity(4, velocity, 1);
}
