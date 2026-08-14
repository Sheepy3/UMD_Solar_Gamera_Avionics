#include "Drone.h"

/*
ArmController armN;
ArmController armE;
ArmController armS;
ArmController armW;

SerialUSB usbSerial;
SerialUART radioSerial;

void setup();
void main();
*/

static const uint32_t CRSF_BAUD = 460800;
static const uint16_t CLARA_FLAG_A = 172;
static const uint16_t CLARA_FLAG_B = 992;
static const uint16_t CLARA_FLAG_C = 1811;
static const uint16_t CLARA_FLAG_TOLERANCE = 50;
static const uint8_t THROTTLE_N_CHANNEL = 0;
static const uint8_t THROTTLE_E_CHANNEL = 1;
static const uint8_t THROTTLE_S_CHANNEL = 2;
static const uint8_t THROTTLE_W_CHANNEL = 3;
static const uint8_t COMMAND_FLAG_CHANNEL = 5;

static bool channelNear(uint16_t value, uint16_t target) {
    return (value > target) ? (value - target <= CLARA_FLAG_TOLERANCE) : (target - value <= CLARA_FLAG_TOLERANCE);
}

static BitFlags decodeControlFlags(uint16_t channelValue) {
    if (channelNear(channelValue, CLARA_FLAG_A)) {
        return BitFlags{
            .id = 0x001,
            .setArm = true,
            .setEStop = false,
            .resetEStop = false,
            .getArm = false,
            .getEStop = false
        };
    }

    if (channelNear(channelValue, CLARA_FLAG_B)) {
        return BitFlags{
            .id = 0x001,
            .setArm = false,
            .setEStop = true,
            .resetEStop = false,
            .getArm = false,
            .getEStop = false
        };
    }

    if (channelNear(channelValue, CLARA_FLAG_C)) {
        return BitFlags{
            .id = 0x001,
            .setArm = false,
            .setEStop = false,
            .resetEStop = true,
            .getArm = false,
            .getEStop = false
        };
    }

    return unpackBitFlags(channelValue);
}

Drone::Drone(DroneParams& params) : armN(params.armNPWMPin, params.armNHallPin),
                                   armE(params.armEPWMPin, params.armEHallPin),
                                   armS(params.armSPWMPin, params.armSHallPin),
                                   armW(params.armWPWMPin, params.armWHallPin),
                                   statusLed(params.statusLedPin),
                                   usbRadio(params.serialParam),
                                   uartRadio(params.radioParam),
                                   debugSerial(params.serialParam)
{
    params.radioParam.setTX(params.txPin);
    params.radioParam.setRX(params.rxPin);
    params.radioParam.setFIFOSize(256);
    params.radioParam.begin(CRSF_BAUD);

    params.serialParam.begin(115200);
}

void Drone::setup()
{
    armN.setup();
    armE.setup();
    armS.setup();
    armW.setup();
    statusLed.setup();

    usbRadio.setCallback([](void* ctx, Radio& source, uint8_t type, uint8_t* payload, uint8_t len) {
        ((Drone*)ctx)->processIncommingFrame(source, type, payload, len);
    }, this);

    uartRadio.setCallback([](void* ctx, Radio& source, uint8_t type, uint8_t* payload, uint8_t len) {
        ((Drone*)ctx)->processIncommingFrame(source, type, payload, len);
    }, this);
}

void Drone::processIncommingFrame(Radio& source, const uint8_t type, const uint8_t* payload, const uint8_t len){
    const char* sourceName = "unknown";
    uint32_t sourceDtMS = 0;

    if (&source == &usbRadio) {
        sourceName = "usb";
        sourceDtMS = (lastUSBRecieveTimeMS == 0) ? 0 : nowMS - lastUSBRecieveTimeMS;
        lastUSBRecieveTimeMS = nowMS;
    }
    else if (&source == &uartRadio) {
        sourceName = "uart";
        sourceDtMS = (lastUARTRecieveTimeMS == 0) ? 0 : nowMS - lastUARTRecieveTimeMS;
        lastUARTRecieveTimeMS = nowMS;

        if (nowMS - lastUSBRecieveTimeMS < TIMEOUT_MS){
            printIncomingCRSF(sourceName, sourceDtMS, type, payload, len, nullptr, true);
            return;
        }
    }

    switch (type)
    {
    case PayloadType::RC_CHANNELS_PACKED:{
        if (len != 22){
            break;
        }
        uint16_t channels[16];

        unpackRCChannels(payload, channels);
        rawThrottleChannels[0] = channels[THROTTLE_N_CHANNEL];
        rawThrottleChannels[1] = channels[THROTTLE_E_CHANNEL];
        rawThrottleChannels[2] = channels[THROTTLE_S_CHANNEL];
        rawThrottleChannels[3] = channels[THROTTLE_W_CHANNEL];

        BitFlags flags = decodeControlFlags(channels[COMMAND_FLAG_CHANNEL]);
        printIncomingCRSF(sourceName, sourceDtMS, type, payload, len, channels, false);

        switch (flags.id){
        case 0x001:
            if (flags.setEStop) {
                triggerEStop("command");
                break;
            }

            if (flags.resetEStop && nowMS - EStopTriggerTimeMS > ESTOP_LOCKOUT_MS){
                EStopActive = false;
            }
            
            armed = !EStopActive && flags.setArm;

            if (EStopActive || !armed){
                armN.stop();
                armE.stop();
                armS.stop();
                armW.stop();
                break;
            }
            
            armN.setThrottle(channelToFloat(channels[THROTTLE_N_CHANNEL]));
            armE.setThrottle(channelToFloat(channels[THROTTLE_E_CHANNEL]));
            armS.setThrottle(channelToFloat(channels[THROTTLE_S_CHANNEL]));
            armW.setThrottle(channelToFloat(channels[THROTTLE_W_CHANNEL]));

            break;
        /*
        case 0x010:
            break;
        case 0x100: //No response
            break;
        */
        default:
            break;
        }
        
        break;
    }
    /*
    case PayloadType::BATTERY_SENSOR:
        break;
    
    case PayloadType::ALTITUDE:
        break;
    
    case PayloadType::LINK_STATISTICS:
        break;
    */
    default:
        printIncomingCRSF(sourceName, sourceDtMS, type, payload, len, nullptr, false);
        break;
    }
}

void Drone::sendTelemetry() {
    // CRSF 0x21 is a standard receiver-to-transmitter telemetry frame. Use
    // its short ASCII flight-mode payload as a deliberately conservative
    // carrier before defining a project-specific binary extended frame.
    // Format: SG1 + sequence(4 hex) + status(2 hex) + uptime-low(4 hex).
    const uint8_t status = (armed ? 0x01 : 0x00) |
                           (EStopActive ? 0x02 : 0x00);
    const uint16_t sequence = telemetrySequence++;
    char payload[16];

    snprintf(
        payload,
        sizeof(payload),
        "SG1%04X%02X%04X",
        static_cast<unsigned int>(sequence),
        static_cast<unsigned int>(status),
        static_cast<unsigned int>(nowMS & 0xFFFF)
    );

    printDebugTelemetry(payload);
    uartRadio.send(
        DestType::GROUND_STATION,
        PayloadType::FLIGHT_MODE,
        reinterpret_cast<const uint8_t*>(payload),
        strlen(payload) + 1
    );

    //TODO: send imu data
}

void Drone::printDebugTelemetry(const char* payload) {
    debugSerial.print("[DEBUG] CRSF_FLIGHT_MODE payload=");
    debugSerial.println(payload);
}

void Drone::main()
{
    while(true){
        nowMS = millis();

        if (nowMS - lastSentTelemetry >= TELEMETRY_DELAY) {
            lastSentTelemetry = nowMS;
            sendTelemetry();
        }
        
        usbRadio.update();
        uartRadio.update();

        if (nowMS - lastUSBRecieveTimeMS > TIMEOUT_MS && nowMS - lastUARTRecieveTimeMS > TIMEOUT_MS){
            triggerEStop("timeout");
        }

        if (armN.isStalled() || armE.isStalled() || armS.isStalled() || armW.isStalled()){
            triggerEStop("stall");
        }

        statusLed.update(getLedStatus());
    }
}

void Drone::triggerEStop(const char* reason){
    EStopActive = true;
    armed = false;
    EStopTriggerTimeMS = nowMS;
    printEStopReason(reason);

    armN.stop();
    armE.stop();
    armS.stop();
    armW.stop();
}

LedStatus Drone::getLedStatus() const {
    if (EStopActive && nowMS - EStopTriggerTimeMS < ESTOP_LOCKOUT_MS) {
        return LedStatus::Lockout;
    }

    if (!armed) {
        return EStopTriggerTimeMS != 0 ? LedStatus::LockoutExitedUnarmed : LedStatus::Disarmed;
    }

    return isThrottleActive() ? LedStatus::ThrottleActive : LedStatus::Armed;
}

bool Drone::isThrottleActive() const {
    return armN.getThrottle() > 0.0f || armE.getThrottle() > 0.0f ||
           armS.getThrottle() > 0.0f || armW.getThrottle() > 0.0f;
}

void Drone::printEStopReason(const char* reason) {
    const bool reasonChanged = lastEStopReason != reason;

    if (!reasonChanged && nowMS - lastEStopLogTimeMS < 500UL) {
        return;
    }

    lastEStopReason = reason;
    lastEStopLogTimeMS = nowMS;

    debugSerial.print("[ESTOP] reason=");
    debugSerial.print(reason);
    debugSerial.print(" uptime_ms=");
    debugSerial.print(nowMS);
    debugSerial.print(" usb_age_ms=");
    debugSerial.print(nowMS - lastUSBRecieveTimeMS);
    debugSerial.print(" uart_age_ms=");
    debugSerial.print(nowMS - lastUARTRecieveTimeMS);
    debugSerial.print(" armed=");
    debugSerial.println(armed ? "true" : "false");
}

void Drone::printIncomingCRSF(const char* sourceName, const uint32_t sourceDtMS, const uint8_t type, const uint8_t* payload, const uint8_t len, const uint16_t* channels, bool ignored) {
    if (!DEBUG_INCOMING_CRSF) {
        return;
    }

    if (nowMS - lastIncomingCRSFLogTimeMS < 100UL) {
        return;
    }

    lastIncomingCRSFLogTimeMS = nowMS;

    debugSerial.print("[RX] source=");
    debugSerial.print(sourceName);
    debugSerial.print(" type=0x");
    printHexByte(type);
    debugSerial.print(" len=");
    debugSerial.print(len);
    debugSerial.print(" source_dt_ms=");
    debugSerial.print(sourceDtMS);
    debugSerial.print(" ignored=");
    debugSerial.print(ignored ? "true" : "false");

    if (channels != nullptr) {
        const BitFlags flags = decodeControlFlags(channels[COMMAND_FLAG_CHANNEL]);

        debugSerial.print(" flags_raw=");
        debugSerial.print(channels[COMMAND_FLAG_CHANNEL]);
        debugSerial.print(" id=");
        debugSerial.print(flags.id);
        debugSerial.print(" setArm=");
        debugSerial.print(flags.setArm ? "true" : "false");
        debugSerial.print(" setEStop=");
        debugSerial.print(flags.setEStop ? "true" : "false");
        debugSerial.print(" resetEStop=");
        debugSerial.print(flags.resetEStop ? "true" : "false");
        debugSerial.print(" channels=[");

        for (uint8_t i = 0; i < 16; ++i) {
            if (i > 0) {
                debugSerial.print(",");
            }
            debugSerial.print(channels[i]);
        }

        debugSerial.print("]");
    }

    debugSerial.print(" payload_hex=");

    for (uint8_t i = 0; i < len; ++i) {
        printHexByte(payload[i]);
    }

    debugSerial.println();
}

void Drone::printHexByte(uint8_t value) {
    if (value < 0x10) {
        debugSerial.print("0");
    }

    debugSerial.print(value, HEX);
}
