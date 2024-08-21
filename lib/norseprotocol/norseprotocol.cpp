#include "norseprotocol.h"

NorseProtocol::NorseProtocol(PinName tx, PinName rx, int baudRate):
    _uart(tx, rx)
{
    _baudRate = baudRate;
    _uart.set_baud(_baudRate);
    _uart.set_format(
        8,                      /* bits */
        BufferedSerial::None,   /* parity */
        1                       /* stop bit */  
    );
}

NorsePacket NorseProtocol::getPacket()
{
    return _packet;
}

bool NorseProtocol::getIsPacketAvilable()
{
    return isPacketAvilable;
}

void NorseProtocol::runCommunication()
{
    read();
    if (isPacketAvilable)
    {
        eventHandler();
    }
}

void NorseProtocol::respondError(uint8_t reasonCode)
{
    uint8_t data[] = {reasonCode};
    generatePacket(EVENT_RESPONSE_ERROR, data, 1);
    write();
}

void NorseProtocol::respondOk(uint8_t eventId)
{
    uint8_t data[] = {eventId};
    generatePacket(EVENT_RESPONSE_OK, data, 1);
    write();
}

void NorseProtocol::respondEventPing()
{
    uint8_t data[] = {PCB_MAJOR_VER, PCB_MINOR_VER, FW_MAJOR_VER, FW_MINOR_VER, PTC_MAJOR_VER, PTC_MINOR_VER};
    generatePacket(EVENT_RESPONSE_DATA, data, 6);
    write();
}

void NorseProtocol::read()
{
    bool isError = false;
    bool isGetReturnChar = false;
    bool isEnding = false;
    uint8_t tmpBuffer[1];
    uint8_t bytes;

    // 1. Get data to buffer
    while (_uart.readable()) 
    {
        if (uint8_t readBytesNumber = _uart.read(tmpBuffer, sizeof(tmpBuffer))) 
        {
            // _uart.write(tmpBuffer, readBytesNumber);

            if (tmpBuffer[0] == '\r' || tmpBuffer[0] == STOP_BYTE_1) 
            {
                isGetReturnChar = true;
            }
            else if ((tmpBuffer[0] == '\n' || tmpBuffer[0] == STOP_BYTE_2) && isGetReturnChar) 
            {
                isEnding = true;
                isGetReturnChar = false;
            }
            else 
            {
                rxBuffer[counter] = tmpBuffer[0];
                counter++;
                isGetReturnChar = false;
            }
        }
    }
    
    // if (counter) {
    if (counter && isEnding) 
    {
        // _uart.write(rxBuffer, counter);
        isReading = true;
        bytes = counter - NUMBER_HEADER - 2;    // 2 is checksum (LSB and MSB)

        // 2. Check header (start byte and protocol version)
        if (!validateHeader()) 
        {
            respondError(ERR_HEADER);
            isError = true;
            // printf("Header error \r\n");
        }

        // 3. Validate checksum
        if (!validateChecksum(rxBuffer, bytes)) 
        {
            respondError(ERR_CHECKSUM);
            isError = true;
            // printf("Checksum error \r\n");
        }

        // 4. Create packet
        if (!isError) 
        {
            _packet.length = rxBuffer[2];
            _packet.eventId = rxBuffer[3];
            for (int i=0; i<(bytes-2); i++) 
            {
                _packet.parameters[i] = rxBuffer[i+4];
            }
            isPacketAvilable = true;
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

void NorseProtocol::generatePacket(uint8_t eventId, uint8_t *params, uint8_t bytesParams)
{
    uint8_t i;
    uint8_t iStart = NUMBER_HEADER + 2;  // 2 = LEN + EID
    uint16_t checksum;

    // 1. Create header
    txBuffer[0] = START_BYTE_1;
    txBuffer[1] = START_BYTE_2;
    // 2. Create lenght
    txBuffer[2] = bytesParams + 3;  // 3 = EID + 2checksum
    // 3. Create event ID and params
    txBuffer[3] = eventId;
    for (i = iStart; i < iStart + bytesParams; i++)
    {
        txBuffer[i] = params[i - iStart];
    }
    // 4. Create checksum
    checksum = calculateChecksum(txBuffer, bytesParams + 1);  // 1 = EID
    txBuffer[i] = checksum & 0xFF;      // LSB
    txBuffer[i + 1] = checksum >> 8;    // MSB
    // 5. Create trailer
    txBuffer[i + 2] = STOP_BYTE_1;
    txBuffer[i + 3] = STOP_BYTE_2;

    lengthTxBuffer = i + 4;
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

bool NorseProtocol::validateHeader()
{
    if ((rxBuffer[0] == START_BYTE_1) && (rxBuffer[1] == START_BYTE_2))
    {
        return true;
    }
    return false;
}

FileHandle *NorseProtocol::mbed_override_console(int fd)
{
    return &_uart;
}

void NorseProtocol::eventHandler()
{
    switch (_packet.eventId)
    {
    case EVENT_PING:    respondEventPing(); break;
    
    default:
        break;
    }
}
