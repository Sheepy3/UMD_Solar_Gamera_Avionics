#include "Radio.h"

Radio::Radio(Stream &s) : serial(s) {}

void Radio::setCallback(Callback cb, void* ctx) { 
    onPacket = cb;
    context = ctx;
}

void Radio::update()
{
    if (state != HUNTING && millis() - lastByteTimeMS > 10){
        state = HUNTING;
    }

    while (serial.available() > 0){
        const uint8_t b = serial.read();
        lastByteTimeMS = millis();

        switch (state){
        case HUNTING:
            if (b == SYNC_BYTE){
                state = GET_LEN;
            }
            break;
        
        case GET_LEN:
            if (b >= 2 && b <= 62){
                expectedLength = b;
                payloadIndex = 0;
                state = GET_PAYLOAD;
            }

            break;

        case GET_PAYLOAD:
            buffer[payloadIndex++] = b;

            if (payloadIndex >= expectedLength){
                const uint8_t recievedCRC = buffer[payloadIndex - 1];
                const uint8_t computedCRC = crc8_d5(buffer, payloadIndex - 1);

                if (recievedCRC == computedCRC && onPacket != nullptr){
                    onPacket(context, *this, buffer[0], &buffer[1], expectedLength - 2);
                }

                state = HUNTING;
            }

            break;
        }
    }
}

uint8_t Radio::crc8_d5(const uint8_t *data, size_t len)
{
    static const uint8_t CRSF_POLY = 0xD5;
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool Radio::send(const uint8_t targetSync, const uint8_t type, const uint8_t* payload, size_t payloadLen){
    if (payloadLen > 60) return false;

    const uint8_t crsfFrameByte = payloadLen + 2;
    const uint8_t totalFrameLen = payloadLen + 4;

    if (serial.availableForWrite() < totalFrameLen){
        return false;
    }

    uint8_t sendBuffer[64]; 
    sendBuffer[0] = targetSync;
    sendBuffer[1] = crsfFrameByte;
    sendBuffer[2] = type;

    if (payload != nullptr && payloadLen > 0) {
        memcpy(&sendBuffer[3], payload, payloadLen);
    }

    uint8_t crc = crc8_d5(&sendBuffer[2], payloadLen + 1);

    sendBuffer[3 + payloadLen] = crc;

    serial.write(sendBuffer, totalFrameLen);
    return true;
}