#include <Arduino.h>
#include "ArmController.h"

// Arm pins
static const int armNPWMPin = 1;
static const int armNHallPin = 1;

static const int armEPWMPin = 1;
static const int armEHallPin = 1;

static const int armSPWMPin = 1;
static const int armSHallPin = 1;

static const int armWPWMPin = 1;
static const int armWHallPin = 1;

// Hardware Configuration
static const int PIN_TX = 16;
static const int PIN_RX = 17;
static const uint32_t CRSF_BAUD = 460800; // Standard CRSF high-speed baud rate

static const uint8_t CRC_POLY = 0xD5;

// CRSF Address definitions
enum DestType : uint8_t {
  REMOTE_CONTROL = 0xEA,
  TRANSMITTER = 0xEE,
  GROUND_STATION = 0xC8,
  GPS = 0xC2 //Unused
};

// CRSF Protocol Constants
enum PayloadType : uint8_t {
  RC_CHANNELS_PACKED = 0x16,
  BATTERY_SENSOR = 0x08,
  ATTITUDE = 0x1E,
  LINK_STATISTICS = 0x14
};

static uint32_t lastBatMs = 0;

static uint8_t crc8_d5(const uint8_t *data, size_t len)
{
  uint8_t crc = 0;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
    {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC_POLY) : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

// Helpers to read Big-Endian (MSB first) values from the byte stream
static uint16_t u16BEtoLE(const uint8_t *p)
{
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

static uint32_t u24BEtoLE(const uint8_t *p)
{
  return (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | uint32_t(p[2]);
}

/**
 * Unpacks the 22-byte RC channel payload into 16 individual 11-bit integers.
 * CRSF packs bits tightly (11 bits per channel) to save bandwidth.
 */
static void unpackRCChannels(const uint8_t *payload, uint16_t out16[16])
{
  uint32_t bits = 0;
  int bitcount = 0;
  int idx = 0;

  for (int i = 0; i < 22; i++)
  {
    bits |= (uint32_t)payload[i] << bitcount;
    bitcount += 8;

    while (bitcount >= 11 && idx < 16)
    {
      out16[idx++] = bits & 0x7FF; // Mask for 11 bits (2047)
      bits >>= 11;
      bitcount -= 11;
    }
  }
}

/**
 * Constructs and sends a Battery telemetry frame via Serial1
 */
static void sendBatteryTo(uint8_t dest, uint16_t voltage_x10, uint16_t current_x10, uint32_t capacity_mah, uint8_t remaining_pct)
{
  // Payload is 8 bytes: Volts(2), Amps(2), Capacity(3), Remaining(1)
  uint8_t payload[8] = {
      (uint8_t)(voltage_x10 >> 8), (uint8_t)(voltage_x10 & 0xFF),
      (uint8_t)(current_x10 >> 8), (uint8_t)(current_x10 & 0xFF),
      (uint8_t)((capacity_mah >> 16) & 0xFF),
      (uint8_t)((capacity_mah >> 8) & 0xFF),
      (uint8_t)(capacity_mah & 0xFF),
      remaining_pct};

  // Frame structure: [Addr] [Len] [Type] [Payload(8)] [CRC]
  // 'Len' includes everything after the Len byte (Type + Payload + CRC = 10)
  uint8_t frame[12];
  frame[0] = dest;
  frame[1] = 10;
  frame[2] = PayloadType::BATTERY_SENSOR;
  memcpy(&frame[3], payload, 8);
  // CRC is calculated starting from the Type byte through the end of the Payload
  frame[11] = crc8_d5(&frame[2], 1 + 8);

  Serial1.write(frame, sizeof(frame));
}

void setup()
{
  //DO NOT ATTACH ANY INTERUPTS IN MAIN
  /*
  _____   ____       _   _  ____ _______            _______ _______       _____ _    _               _   ___     __     _____ _   _ _______ ______ _____  _    _ _____ _______ _____      _____ _   _      __  __          _____ _   _ 
 |  __ \ / __ \     | \ | |/ __ \__   __|        /\|__   __|__   __|/\   / ____| |  | |        /\   | \ | \ \   / /    |_   _| \ | |__   __|  ____|  __ \| |  | |  __ \__   __/ ____|    |_   _| \ | |    |  \/  |   /\   |_   _| \ | |
 | |  | | |  | |    |  \| | |  | | | |          /  \  | |     | |  /  \ | |    | |__| |       /  \  |  \| |\ \_/ /       | | |  \| |  | |  | |__  | |__) | |  | | |__) | | | | (___        | | |  \| |    | \  / |  /  \    | | |  \| |
 | |  | | |  | |    | . ` | |  | | | |         / /\ \ | |     | | / /\ \| |    |  __  |      / /\ \ | . ` | \   /        | | | . ` |  | |  |  __| |  _  /| |  | |  ___/  | |  \___ \       | | | . ` |    | |\/| | / /\ \   | | | . ` |
 | |__| | |__| |    | |\  | |__| | | |        / ____ \| |     | |/ ____ \ |____| |  | |     / ____ \| |\  |  | |        _| |_| |\  |  | |  | |____| | \ \| |__| | |      | |  ____) |     _| |_| |\  |    | |  | |/ ____ \ _| |_| |\  |
 |_____/ \____/     |_| \_|\____/  |_|       /_/    \_\_|     |_/_/    \_\_____|_|  |_|    /_/    \_\_| \_|  |_|       |_____|_| \_|  |_|  |______|_|  \_\\____/|_|      |_| |_____/     |_____|_| \_|    |_|  |_/_/    \_\_____|_| \_|
                                                                                                                                                                                                                                                                                                                                                                                                                                                               
  */
  delay(500);
  Serial.begin(115200); // USB Debug output
  Serial.println("RP2040 CRSF sniffer + battery injector");

  // Configure Hardware Serial for CRSF
  Serial1.setTX(PIN_TX);
  Serial1.setRX(PIN_RX);
  Serial1.begin(CRSF_BAUD);

  Serial.print("Serial1 @ ");
  Serial.println(CRSF_BAUD);
}

void loop()
{
  // ----- SECTION 1: TIMED TELEMETRY INJECTION (10Hz) -----
  

  uint32_t nowMs = millis();
  if (nowMs - lastBatMs >= 100) // Trigger every 100ms
  {
    lastBatMs = nowMs;
  }

  // ----- SECTION 2: PARSE INCOMING CRSF FRAMES -----
  static uint8_t buffer[256];
  static size_t bufferLength = 0;

  // Read available bytes into the buffer
  while (Serial1.available() && bufferLength < sizeof(buffer))
  {
    buffer[bufferLength++] = (uint8_t)Serial1.read();
  }

  // Process the buffer
  while (bufferLength >= 4) // Smallest possible CRSF frame is 4 bytes
  {
    uint8_t address = buffer[0];
    uint8_t packetLength = buffer[1];

    // Validate length: CRSF frames (including Type/CRC) are 2 to 64 bytes
    if (packetLength < 2 || packetLength > 64)
    {
      // Not a valid frame start, shift buffer by 1 and try again
      memmove(buffer, buffer + 1, --bufferLength);
      continue;
    }

    size_t frame_len = 2 + packetLength; // Total frame: Addr + Len + (Payload contents)
    if (bufferLength < frame_len)
    {
      break; // Full frame hasn't arrived yet, wait for more data
    }

    uint8_t type = buffer[2];
    uint8_t *payload = &buffer[3];
    size_t payload_len = packetLength - 2; // Subtract Type and CRC bytes
    uint8_t crc = buffer[frame_len - 1];

    // Verify CRC (Header byte not included in CRC, starts from Type)
    uint8_t calc = crc8_d5(&buffer[2], 1 + payload_len);
    if (calc != crc)
    {
      // CRC mismatch, shift and look for next valid header
      memmove(buffer, buffer + 1, --bufferLength);
      continue;
    }

    // ----- SECTION 3: DECODE KNOWN FRAME TYPES -----

    if (type == PayloadType::LINK_STATISTICS && payload_len >= 4)
    {
      int8_t rssi1 = (int8_t)payload[0]; // RSSI Ant 1 (dBm)
      int8_t rssi2 = (int8_t)payload[1]; // RSSI Ant 2 (dBm)
      uint8_t lq = payload[2];           // Link Quality (%)
      int8_t snr = (int8_t)payload[3];   // SNR (dB)
      Serial.print("LINK address=0x");
      Serial.print(address, HEX);
      Serial.print(" RSSI=");
      Serial.print(rssi1);
      Serial.print("/");
      Serial.print(rssi2);
      Serial.print(" LQ=");
      Serial.print(lq);
      Serial.print("% SNR=");
      Serial.println(snr);
    }
    else if (type == PayloadType::RC_CHANNELS_PACKED && payload_len >= 22)
    {
      uint16_t ch[16];
      unpackRCChannels(payload, ch);
      Serial.print("RC address=0x");
      Serial.print(address, HEX);
      Serial.print(" ch1=");
      Serial.print(ch[0]); // Range: 172 to 1811 (Center: 992)
      Serial.print(" ch2=");
      Serial.print(ch[1]);
      Serial.print(" ch3=");
      Serial.print(ch[2]);
      Serial.print(" ch4=");
      Serial.print(ch[3]);
      Serial.print(" ... ch16=");
      Serial.println(ch[15]);
    }
    else if (type == PayloadType::BATTERY_SENSOR && payload_len >= 8)
    {
      float v = u16BEtoLE(&payload[0]) / 10.0f; // Voltage in 0.1V
      float a = u16BEtoLE(&payload[2]) / 10.0f; // Current in 0.1A
      uint32_t cap = u24BEtoLE(&payload[4]);    // Capacity in mAh
      uint8_t pct = payload[7];              // Remaining %
      Serial.print("BAT address=0x");
      Serial.print(address, HEX);
      Serial.print(" ");
      Serial.print(v, 1);
      Serial.print("V ");
      Serial.print(a, 1);
      Serial.print("A ");
      Serial.print(cap);
      Serial.print("mAh ");
      Serial.print((int)pct);
      Serial.println("%");
    }

    // Remove the processed frame from the buffer and repeat
    memmove(buffer, buffer + frame_len, bufferLength - frame_len);
    bufferLength -= frame_len;
  }

  delay(1); // Small yield for system stability
}