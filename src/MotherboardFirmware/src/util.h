#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

struct BitFlags {
    uint8_t id;
    bool setArm;
    bool setEStop;
    bool resetEStop;
    bool getArm;
    bool getEStop;
};

enum FlagMasks : uint16_t {
    ID_MASK      = 0b111,
    SET_ARM      = (1U << 3),
    SET_ESTOP    = (1U << 4),
    RESET_ESTOP  = (1U << 5),
    GET_ARM      = (1U << 6),
    GET_ESTOP    = (1U << 7)
};

inline BitFlags unpackBitFlags(uint16_t data) {
    return BitFlags{
        .id         = static_cast<uint8_t>(data & ID_MASK),
        .setArm     = (data & SET_ARM) != 0,
        .setEStop   = (data & SET_ESTOP) != 0,
        .resetEStop = (data & RESET_ESTOP) != 0,
        .getArm     = (data & GET_ARM) != 0,
        .getEStop   = (data & GET_ESTOP) != 0
    };
}

inline void unpackRCChannels(const uint8_t *payload, uint16_t out16[16])
{
    uint32_t bits = 0;
    uint8_t bitcount = 0;
    uint8_t idx = 0;

    for (uint8_t i = 0; i < 22; i++)
    {
        bits |= (uint32_t)payload[i] << bitcount;
        bitcount += 8;

        while (bitcount >= 11 && idx < 16)
        {
        out16[idx++] = bits & 0x7FF;
        bits >>= 11;
        bitcount -= 11;
        }
    }
}

inline float channelToFloat(uint16_t data) {
    uint16_t maskedValue = data & 0x07FF;
    
    return static_cast<float>(maskedValue) / 2047.0f;
}

inline uint16_t u16BEtoLE(const uint8_t *p) 
{
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

inline uint32_t u24BEtoLE(const uint8_t *p)
{
    return (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | uint32_t(p[2]);
}

#endif