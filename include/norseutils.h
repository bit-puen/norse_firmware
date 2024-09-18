#ifndef NORSE_UTILS_H
#define NORSE_UTILS_H

#define BYTES2UINT16(byts_h, byts_l)    uint16_t(uint16_t(byts_h) << 8 | uint16_t(byts_l))
#define BYTES2INT16(byts_h, byts_l)     int16_t(int16_t(byts_h) << 8 | int16_t(byts_l))

#define DEG2RAD(DEG)                    DEG * 0.01745
#define LSB2DEGREE(LSB)                 LSB * 0.08789
#define LSB2RPM(LSB)                    LSB * 0.22900
#define RPM2LSB(RPM)                    RPM * 4.36681
#define RPM2RADPERSEC(RPM)              RPM * 0.10472  //rad/s
#define RADPERSEC2RPM(RAD)              RAD * 9.549297

#endif