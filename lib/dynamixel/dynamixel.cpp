#include "dynamixel.h"

Dynamixel::Dynamixel(PinName tx, PinName rx, uint32_t baudRate, PinName rs485DirectionPin): 
    _dynamixel(tx, rx), _rs485DirectionPin(rs485DirectionPin)
{
    _baudRate = baudRate;
    _dynamixel.set_baud(_baudRate);
    _dynamixel.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
    _responseTimeoutMicroSec = DEFAULT_RESP_TIMEOUT_US;
}

void Dynamixel::enableTorque(uint8_t id)
{
    uint8_t data[1];
    data[0] = 1;
    writeRegister(id, REG_TORQUE_ENABLE, 1, data);
    wait_us(DELAY_TIME_US);
}

void Dynamixel::disableTorque(uint8_t id)
{
    uint8_t data[1];
    data[0] = 0;
    writeRegister(id, REG_TORQUE_ENABLE, 1, data);
    wait_us(DELAY_TIME_US);
}

void Dynamixel::enableLed(uint8_t id)
{
    uint8_t data[1];
    data[0] = 1;
    writeRegister(id, REG_LED_ENABLE, 1, data);
    wait_us(DELAY_TIME_US);
}

void Dynamixel::disableLed(uint8_t id)
{
    uint8_t data[1];
    data[0] = 0;
    writeRegister(id, REG_LED_ENABLE, 1, data);
    wait_us(DELAY_TIME_US);
}

void Dynamixel::setGoalVelocity(uint8_t id, int goalVelocity)
{
    uint8_t velocity[4];

    velocity[0] = goalVelocity & 0xff;
    velocity[1] = (goalVelocity >> 8) & 0xff;
    velocity[3] = (goalVelocity >> 16) & 0xff;
    velocity[4] = goalVelocity >> 24;

    writeRegister(id, REG_GOAL_VELOCITY, 4, velocity);
    wait_us(DELAY_TIME_US);
}

void Dynamixel::setGoalVelocity(uint8_t id, uint16_t goalVelocity, uint8_t direction)
{
    uint8_t velocity[4];
    uint16_t speed = (((direction - 1) * goalVelocity) + (direction * goalVelocity));
    uint16_t first_bit = ((1 - direction) * 0xff) + (direction * 0x00);

    velocity[0] = speed & 0xff;
    velocity[1] = speed >> 8;
    velocity[2] = first_bit;
    velocity[3] = first_bit;

    writeRegister(id, REG_GOAL_VELOCITY, 4, velocity);
    wait_us(DELAY_TIME_US);
}

int Dynamixel::getPresentPosition(uint8_t id)
{
    // Timer timeoutTimer;
    int presentPosition;
    StatusCode packetStatus;
    // uint8_t recursiveCount = 0;

    packetStatus = readRegister(id, REG_PRESENT_POSITION, LEN_PRESENT_POSITION, presentPosition);
    
    // if (packetStatus != packetResultOk)
    // {
    //     // timeoutTimer.start();
    //     // packetStatus = readRegister(id, REG_PRESENT_POSITION, LEN_PRESENT_POSITION, presentPosition);
    //     // while (packetStatus != packetResultOk && timeoutTimer.read_us() < 500)
    //     // while (packetStatus != packetResultOk && recursiveCount < 10)
    //     // {
    //     //     packetStatus = readRegister(id, REG_PRESENT_POSITION, LEN_PRESENT_POSITION, presentPosition);
    //     //     printf("fail: %d\r\n", packetStatus);
    //     //     recursiveCount++;
    //     // }
    //     // timeoutTimer.stop();
    // }

    // printf("presentPosition: %d \r\n", presentPosition);
    wait_us(DELAY_TIME_US);
    return presentPosition;
}

int Dynamixel::getPresentVelocity(uint8_t id)
{
    int presentVelocity;
    StatusCode packetStatus;
    packetStatus = readRegister(id, REG_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY, presentVelocity);
    printf("presentVelocity: %d\r\n", presentVelocity);
    wait_us(DELAY_TIME_US);
    return presentVelocity;
}

StatusCode Dynamixel::writeRegister(uint8_t id, uint16_t registerAddress, uint16_t n_params, uint8_t *data)
{
    write(id, INSTRUCTION_WRITE_DATA, registerAddress, n_params, data);
    return read(0);
}

void Dynamixel::write(uint8_t id, uint8_t instruction, uint16_t registerAddress, uint16_t n_params, uint8_t *data)
{
    // 0xff, 0xff, 0xfd, 0x00, ID, Len_L, Len_H, Instruction, Address, Param(s), CRC_L, CRC_H
    // length = n_params + 3 (Instruction: 1byte, CRC: 2bytes)
    // n_params = n_params + 2 (registerAddress: 2bytes)
    uint16_t txDelayTimeMicroSec;
    uint8_t length = n_params + 5;
    uint8_t txBuffer[length + 7];
    unsigned short crc = 0;

    txBuffer[0] = 0xFF;                     // Header1
    txBuffer[1] = 0xFF;                     // Header2
    txBuffer[2] = 0xFD;                     // Header3
    txBuffer[3] = 0x00;                     // Reserved
    txBuffer[4] = id;                       // Packet ID
    txBuffer[5] = length & 0xFF;            // Len_L // bottom 8 bits //Packet Length = number of Parameters + 3
    txBuffer[6] = length >> 8;              // Len_H // top 8 bits //
    txBuffer[7] = instruction;              // Instruction
    txBuffer[8] = registerAddress & 0xFF;   // Address L
    txBuffer[9] = registerAddress >> 8;     // Address H

    for (int i=0; i<n_params ; i++) {
        txBuffer[10+i] = data[i];
    }

    crc = update_crc(0, txBuffer, length + 5);

    txBuffer[10+n_params] = crc & 0x00ff;         // CRC_L // CRC_L = (CRC & 0x00FF);
    txBuffer[11+n_params] = (crc >> 8) & 0x00ff;  // CRC_H // first dummy //

    // Debug
    /* 
    printf("TX: ");
    for (uint8_t i = 0; i < length + 7; i++)
    {
        printf("%d ", txBuffer[i]);
    }
    printf("\r\n");
    */

    //send Instruction Packet
    _rs485DirectionPin = 1;
    //wait for 74HC241 enable pin transition delay  
    // 5 microsec estimated by oscilloscope @puen
    wait_us(5);
    _dynamixel.write(txBuffer, sizeof(txBuffer));
    // Wait for data to transmit
    txDelayTimeMicroSec = int((sizeof(uint8_t) * (length + 7) * 8 * 1.2) * (1000000 / _baudRate)) + DELAY_TX_OFFSET_US;
    wait_us(txDelayTimeMicroSec);
    // wait_us(110);
    _rs485DirectionPin = 0;
}

StatusCode Dynamixel::readRegister(uint8_t id, uint16_t registerAddress, uint16_t bytes, int& data)
{
    uint8_t params[2];
    params[0] = bytes & 0xFF;
    params[1] = bytes >> 8;

    write(id, INSTRUCTION_READ_DATA, registerAddress, 2, params);

    return read(bytes, data);
}

StatusCode Dynamixel::read(uint16_t bytesStuatusParams)
{
    int dummyData;
    return read(bytesStuatusParams, dummyData);
}

StatusCode Dynamixel::read(uint16_t bytesStuatusParams, int& data)
{
    // Timer timeoutTimer;
    // uint8_t counterBuffer = 0;
    // uint8_t tmpBuffer[bytesStuatusParams + 11];
    uint8_t rxBuffer[bytesStuatusParams + 11];
    uint8_t readBytesNumber;
    uint16_t packetLength;
    // [Header1][Header2][Header3][Reserved][PacketID][Length1][Length2][Instruction][ERR][CRC1][CRC2]
    // uint16_t lenPacket = 11 + bytesStuatusParams;

    // timeoutTimer.start();
    // while ((timeoutTimer.read_us() < _responseTimeoutMicroSec) && _dynamixel.readable())
    // {
    readBytesNumber = _dynamixel.read(rxBuffer, sizeof(rxBuffer));
        // printf("%d ", readBytesNumber);
        // if (readBytesNumber)
        // {
        //     rxBuffer[counterBuffer] = tmpBuffer[0];
        //     counterBuffer++;
        //     timeoutTimer.reset();
        // }
    // }
    // timeoutTimer.stop();

    // Debug
    /*
    timeoutTimer.start();
    printf("RX: ");
    for (size_t i = 0; i < readBytesNumber; i++)
    {
        printf("%d ", rxBuffer[i]);
    }
    timeoutTimer.stop();
    printf(" Time: %llu\r\n", timeoutTimer.elapsed_time().count());
    */

   // wait for storing buffer 
   // 200 microsec estimated by oscilloscope @puen
   wait_us(200); 

    // validate header
    if (rxBuffer[0] != 0xFF && rxBuffer[1] != 0xFF && rxBuffer[2] != 0xFD)
    {
        data = 0;
        return packetResultFail;
    }

    // check packet length
    packetLength = (((uint16_t)rxBuffer[6]) << 8) | (uint16_t)rxBuffer[5];
    if ((packetLength + 7) > readBytesNumber)
    {
        data = 0;
        return packetDataLengthError;
    }

    // check status packet byte
    if (rxBuffer[8] != packetResultOk)
    {
        data = 0;
        return StatusCode(rxBuffer[8]);
    }
    
    switch (bytesStuatusParams)
    {
    case 0:
        data = 0;
        break;
    case 1:
        data = (uint16_t)rxBuffer[9];
        break;
    case 2:
        data = (((uint16_t)rxBuffer[10]) << 8) | (uint16_t)rxBuffer[9];
        break;
    case 4:
        data = (((uint16_t)rxBuffer[12]) << 24) | (((uint16_t)rxBuffer[11]) << 16) | (((uint16_t)rxBuffer[10]) << 8) | (uint16_t)rxBuffer[9];
        break;
    default:
        data = 0;
        break;
    }
    return StatusCode(rxBuffer[8]);
}

unsigned short Dynamixel::update_crc(unsigned short crc_accum, unsigned char *data_blk_prt, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_prt[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}


