#ifdef MK_I
// #define BUADRATE_DYNAMIXEL          1000000
#define BUADRATE_DYNAMIXEL          115200
#define BUADRATE_PROTOCOL           115200
#endif

#define DEG2RAD(DEG)                DEG * 0.01745
#define LSB2DEGREE(LSB)             LSB * 0.08789
#define LSB2RPM(LSB)                LSB * 0.22900
#define RPM2LSB(RPM)                RPM * 4.36681
#define RPM2RADPERSEC(RPM)          RPM * 0.10472  //rad/s
#define RADPERSEC2RPM(RAD)          RAD * 9.549297 