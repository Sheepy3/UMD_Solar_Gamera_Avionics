#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>

typedef void (*Callback)(Radio& source, uint8_t type, uint8_t* payload, uint8_t len);
enum RadioState { HUNTING, GET_LEN, GET_PAYLOAD };

class Radio {
public:
    Radio(Stream& s);
    void setCallback(Callback cb);
    void update();
    bool send(const uint8_t targetSync, const uint8_t type, const uint8_t* payload, size_t payloadLen);
private:
    Stream& serial;
    Callback onPacket = nullptr;
    RadioState state = HUNTING;

    uint8_t buffer[64];
    uint8_t payloadIndex = 0;
    uint8_t expectedLength = 0;
    uint8_t lastByteTimeMS = 0;

    static const uint8_t SYNC_BYTE = 0xC8;

    static uint8_t crc8_d5(const uint8_t* data, size_t len);
};

#endif