#include "norseprotocol.h"

NorseProtocol::NorseProtocol(PinName tx, PinName rx, uint64_t baudRate):
    _uart(tx, rx)
{
    _baudRate = baudRate;
    _uart.set_baud(_baudRate);
    _uart.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
}

NorsePacket NorseProtocol::getPacket()
{
    return _packet;
}

bool NorseProtocol::getIsPacketAvilable()
{
    return false;
}

void NorseProtocol::runCommunication()
{
    this->read();
    // if (uint32_t num = _uart.read(rxBuffer, sizeof(rxBuffer)))
    // {   
        // _uart.write(rxBuffer, num);
    // }
}

void NorseProtocol::read()
{
    bool isError = false;
    bool isGetReturnChar = false;
    bool isEnding = false;
    uint8_t tmpBuffer[1];
    uint8_t readBytesNumber, bytes;
    uint16_t checkSum;

    // 1. Get data to buffer
    while (_uart.readable()) 
    {
        readBytesNumber = _uart.read(tmpBuffer, sizeof(tmpBuffer));
        if (readBytesNumber) 
        {
            // printf("%d ", tmpBuffer[0]);
            printf("%d ", 0x5A);
            if (tmpBuffer[0] == '\r' || tmpBuffer[0] == 13) 
            {
                isGetReturnChar = true;
                printf("r ");
            }
            else if ((tmpBuffer[0] == '\n' || tmpBuffer[0] == 0x0A) && isGetReturnChar) 
            {
                isEnding = true;
                isGetReturnChar = false;
                printf("n");
                break;
            }
            else 
            {
                rxBuffer[counter] = tmpBuffer[0];
                printf("%d ", rxBuffer[counter]);
                counter++;
                isGetReturnChar = false;
            }
        }
    }
    
    // if (counter) {
    if (counter && isEnding) 
    {
        printf("\r\n");
        isReading = true;
        bytes = counter - NUMBER_HEADER - 2;    // 2 is checksum (LSB and MSB)

        // 2. Check header (start byte and protocol version)
        if (!validateHeader()) 
        {
            // noticError(DEVICE_F411RE, 0x00);
            isError = true;
            printf("Header error \r\n");
        }
        // 3. Validate checksum
        if (!validateChecksum(rxBuffer, bytes)) 
        {
            // noticError(DEVICE_F411RE, 0x00);
            isError = true;
            printf("Checksum error \r\n");

        }
        // 4. Create packet
        if (!isError) 
        {
            _packet.length = rxBuffer[2];
            _packet.eventId = rxBuffer[3];
            for (int i=0; i<(bytes-2); i++) 
            {
                _packet.parameters[i] = rxBuffer[i+4];
                // printf("%d+6: %d\n", i, RxBuffer[i+6]);
            }
            // iPacket.parameters[0] = RxBuffer[6];
            isPacketAvilable = true;
            // printf("Perfect packet \r\n");
        }
        else 
        {
            isPacketAvilable = false;
        }
        isReading = false;
        isEnding = false;
        counter = 0;
    }
    else 
    {
        isPacketAvilable = false;
        isReading = false;
    }
}

void NorseProtocol::write()
{
    _uart.write(txBuffer, sizeof(uint8_t) * lengthTxBuffer);
}

bool NorseProtocol::validateChecksum(uint8_t *packet, uint8_t bytes)
{
    uint8_t length = bytes + NUMBER_HEADER;
    uint16_t packetChecksum;
    uint16_t calculatedChecksum = calculateChecksum(packet, bytes);
    
    packetChecksum = (((uint16_t)packet[length + 1]) << 8) | (uint16_t)packet[length];
    
    if (packetChecksum != calculatedChecksum) 
    {
        // errorIndex = ERROR_INVALID_CHECKSUM;
        return false;
    }
    return true;
}

uint16_t NorseProtocol::calculateChecksum(uint8_t *packet, uint8_t bytes)
{
    uint8_t length = bytes + NUMBER_HEADER;
    uint16_t sum = 0;

    for (uint8_t i = NUMBER_HEADER; i < length; i++) 
    {
        sum = sum + packet[i];
    }

    return sum;
}

FileHandle *NorseProtocol::mbed_override_console(int fd)
{
    return &_uart;
}

bool NorseProtocol::validateHeader()
{
    if ((rxBuffer[0] == START_BYTE_L) && (rxBuffer[1] == START_BYTE_H))
    {
        return true;
    }
    return false;
}

