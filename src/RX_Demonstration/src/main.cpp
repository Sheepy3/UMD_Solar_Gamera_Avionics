#include <Arduino.h>

static const int PIN_TX = 16;
static const int PIN_RX = 17;
static const uint32_t CRSF_BAUD = 460800;

static const uint8_t CRC_POLY = 0xD5;

// Frame types
static const uint8_t TYPE_BATTERY          = 0x08;
static const uint8_t TYPE_LINK_STATS       = 0x14;
static const uint8_t TYPE_RC_CHANNELS_PACKED = 0x16;

// Destination addresses to try for battery injection
static const uint8_t ADDR_RX = 0xEA;
//static const uint8_t ADDR_TX = 0xEE;

static uint8_t crc8_d5(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC_POLY) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static uint16_t u16_be(const uint8_t* p) { return (uint16_t(p[0]) << 8) | uint16_t(p[1]); }
static uint32_t u24_be(const uint8_t* p) { return (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | uint32_t(p[2]); }

static void unpack_rc_channels_11bit(const uint8_t* payload, uint16_t out16[16]) {
  uint32_t bits = 0;
  int bitcount = 0;
  int idx = 0;

  for (int i = 0; i < 22; i++) {
    bits |= (uint32_t)payload[i] << bitcount;
    bitcount += 8;
    while (bitcount >= 11 && idx < 16) {
      out16[idx++] = bits & 0x7FF;
      bits >>= 11;
      bitcount -= 11;
    }
  }
}

static void sendBatteryTo(uint8_t dest,
                          uint16_t voltage_x10,
                          uint16_t current_x10,
                          uint32_t capacity_mah,
                          uint8_t remaining_pct) {
  uint8_t payload[8] = {
    (uint8_t)(voltage_x10 >> 8), (uint8_t)(voltage_x10 & 0xFF),
    (uint8_t)(current_x10 >> 8), (uint8_t)(current_x10 & 0xFF),
    (uint8_t)((capacity_mah >> 16) & 0xFF),
    (uint8_t)((capacity_mah >> 8) & 0xFF),
    (uint8_t)(capacity_mah & 0xFF),
    remaining_pct
  };

  // len = type(1) + payload(8) + crc(1) = 10
  uint8_t frame[12];
  frame[0] = dest;
  frame[1] = 10;
  frame[2] = TYPE_BATTERY;
  memcpy(&frame[3], payload, 8);
  frame[11] = crc8_d5(&frame[2], 1 + 8);

  Serial1.write(frame, sizeof(frame));
}

void setup() {
  delay(500);
  Serial.begin(115200);
  Serial.println("RP2040 CRSF sniffer + battery injector");

  Serial1.setTX(PIN_TX);
  Serial1.setRX(PIN_RX);
  Serial1.begin(CRSF_BAUD);

  Serial.print("Serial1 @ ");
  Serial.println(CRSF_BAUD);
}

void loop() {
  // ----- TX battery at 10Hz -----
  static uint32_t lastBatMs = 0;
  static uint16_t v_x10 = 160;   // 16.0V
  static uint32_t mah = 100;

  uint32_t nowMs = millis();
  if (nowMs - lastBatMs >= 100) {
    lastBatMs = nowMs;

    // Send battery frame to RX 
    sendBatteryTo(ADDR_RX, v_x10, 12, mah, 50);
  
    v_x10 = (v_x10 >= 170) ? 160 : (uint16_t)(v_x10 + 1);
    mah += 10;
  }

  // ----- RX parse/print frames -----
  static uint8_t buf[256];
  static size_t blen = 0;

  while (Serial1.available() && blen < sizeof(buf)) {
    buf[blen++] = (uint8_t)Serial1.read();
  }

  while (blen >= 4) {// Minimum frame is 4 bytes: addr(1) + len(1) + type(1) + crc(1)
    uint8_t addr = buf[0];
    uint8_t ln   = buf[1];

    if (ln < 2 || ln > 64) { memmove(buf, buf + 1, --blen); continue; } // Invalid length, skip this byte and try again

    size_t frame_len = 2 + ln;
    if (blen < frame_len) break;

    uint8_t type = buf[2];
    uint8_t* payload = &buf[3];
    size_t payload_len = ln - 2;
    uint8_t crc = buf[frame_len - 1];

    uint8_t calc = crc8_d5(&buf[2], 1 + payload_len);
    if (calc != crc) { memmove(buf, buf + 1, --blen); continue; } // Invalid CRC, skip this byte and try again

    if (type == TYPE_LINK_STATS && payload_len >= 4) { // Link stats frame
      int8_t rssi1 = (int8_t)payload[0];
      int8_t rssi2 = (int8_t)payload[1];
      uint8_t lq   = payload[2];
      int8_t snr   = (int8_t)payload[3];
      Serial.print("LINK addr=0x"); Serial.print(addr, HEX);
      Serial.print(" RSSI="); Serial.print(rssi1); Serial.print("/"); Serial.print(rssi2);
      Serial.print(" LQ="); Serial.print(lq);
      Serial.print("% SNR="); Serial.println(snr);
    }
    else if (type == TYPE_RC_CHANNELS_PACKED && payload_len >= 22) { // RC channels frame
      uint16_t ch[16];
      unpack_rc_channels_11bit(payload, ch);
      Serial.print("RC addr=0x"); Serial.print(addr, HEX);
      Serial.print(" ch1="); Serial.print(ch[0]);
      Serial.print(" ch2="); Serial.print(ch[1]);
      Serial.print(" ch3="); Serial.print(ch[2]);
      Serial.print(" ch4="); Serial.print(ch[3]);
      Serial.print(" ... ch16="); Serial.println(ch[15]);
    }
    else if (type == TYPE_BATTERY && payload_len >= 8) { // Battery frame
      float v = u16_be(&payload[0]) / 10.0f;
      float a = u16_be(&payload[2]) / 10.0f;
      uint32_t cap = u24_be(&payload[4]);
      uint8_t pct = payload[7];
      Serial.print("BAT addr=0x"); Serial.print(addr, HEX);
      Serial.print(" "); Serial.print(v, 1); Serial.print("V ");
      Serial.print(a, 1); Serial.print("A ");
      Serial.print(cap); Serial.print("mAh ");
      Serial.print((int)pct); Serial.println("%");
    }

    memmove(buf, buf + frame_len, blen - frame_len); // Remove this frame from buffer
    blen -= frame_len; // Loop to try parsing next frame in buffer (if any)
  }

  delay(1);
}
