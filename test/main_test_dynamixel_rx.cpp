#include "dynamixel.h"

BufferedSerial serial(USBTX, USBRX, 115200);
Dynamixel XL430(D8, D2, 1000000, D7);

FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial;
}

int main()
{
    while (true)
    {
        // printf("Enable LED \r\n");
        // XL430.enableLed(252);
        // XL430.setGoalVelocity(252, 265, 0);
        // ThisThread::sleep_for(2s);
        // printf("\r\n");

        // printf("Disable LED \r\n");
        // XL430.disableLed(252);
        printf("Position: %d", XL430.getPresentPosition(252));
        ThisThread::sleep_for(5s);
        printf("\r\n");
    }
    return 0;
}