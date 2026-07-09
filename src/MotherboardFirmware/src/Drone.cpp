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

        BitFlags flags = decodeControlFlags(channels[0]);
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
            
            armN.setThrottle(channelToFloat(channels[1]));
            armE.setThrottle(channelToFloat(channels[2]));
            armS.setThrottle(channelToFloat(channels[3]));
            armW.setThrottle(channelToFloat(channels[4]));

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
    uint16_t values[16] = {0};

    const BitFlags flags{
        .id         = 0b100,
        .setArm     = false,
        .setEStop   = false,
        .resetEStop = false,
        .getArm     = armed,
        .getEStop   = EStopActive
    };

    values[0] = packBitFlags(flags);

    values[1] = floatToChannel(armN.getRPM());
    values[2] = floatToChannel(armE.getRPM());
    values[3] = floatToChannel(armS.getRPM());
    values[4] = floatToChannel(armW.getRPM());

    values[5] = floatToChannel(armN.getThrottle());
    values[6] = floatToChannel(armE.getThrottle());
    values[7] = floatToChannel(armS.getThrottle());
    values[8] = floatToChannel(armW.getThrottle());

    values[9] = static_cast<uint16_t>((ESTOP_LOCKOUT_MS + EStopTriggerTimeMS - nowMS) / 100U);

    uint32_t end = EStopTriggerTimeMS + ESTOP_LOCKOUT_MS;

    values[9] = (nowMS < end) ? static_cast<uint16_t>(min((end - nowMS) / 100U, 2047U)) : 0;

    values[14] = static_cast<uint16_t>(nowMS & 0x7FF);
    values[15] = static_cast<uint16_t>((nowMS >> 11) & 0x7FF);

    uint8_t payload[22];

    packRCChannels(values, payload);

    printDebugTelemetry(values);
    uartRadio.send(DestType::GROUND_STATION, 0x16, payload, 22);

    //TODO: send imu data
}

void Drone::printDebugTelemetry(const uint16_t values[16]) {
    if (DEBUG_RAW_CRSF_CHANNELS) {
        debugSerial.print("[DEBUG] CRSF_CHANNELS [");

        for (uint8_t i = 0; i < 16; ++i) {
            if (i > 0) {
                debugSerial.print(", ");
            }
            debugSerial.print(values[i]);
        }

        debugSerial.println("]");
        return;
    }

    const BitFlags flags = unpackBitFlags(values[0]);
    const uint32_t uptimeMS =
        (static_cast<uint32_t>(values[15]) << 11) | values[14];

    debugSerial.print("[DEBUG] armed=");
    debugSerial.print(flags.getArm ? "true" : "false");
    debugSerial.print(" estop=");
    debugSerial.print(flags.getEStop ? "true" : "false");
    debugSerial.print(" lockout_ms=");
    debugSerial.print(static_cast<uint32_t>(values[9]) * 100U);

    debugSerial.print(" rpm=[");
    for (uint8_t i = 1; i <= 4; ++i) {
        if (i > 1) debugSerial.print(",");
        debugSerial.print(values[i]);
    }

    debugSerial.print("] throttle=[");
    for (uint8_t i = 5; i <= 8; ++i) {
        if (i > 5) debugSerial.print(",");
        debugSerial.print(values[i]);
    }

    debugSerial.print("] uptime_ms=");
    debugSerial.println(uptimeMS);
}

void Drone::main()
{
    while(true){
        nowMS = millis();

        if (nowMS - lastSentTelemetry > TELEMETRY_DELAY) {
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
        const BitFlags flags = decodeControlFlags(channels[0]);

        debugSerial.print(" flags_raw=");
        debugSerial.print(channels[0]);
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
