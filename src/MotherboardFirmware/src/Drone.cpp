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

Drone::Drone(DroneParams& params) : armN(params.armNPWMPin, params.armNHallPin),
                                   armE(params.armEPWMPin, params.armEHallPin),
                                   armS(params.armSPWMPin, params.armSHallPin),
                                   armW(params.armWPWMPin, params.armWHallPin),
                                   usbRadio(params.serialParam),
                                   uartRadio(params.radioParam)
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
    if (&source == &usbRadio) {
        lastUSBRecieveTimeMS = nowMS;
    }
    else if (&source == &uartRadio) {
        lastUARTRecieveTimeMS = nowMS;

        if (nowMS - lastUSBRecieveTimeMS < TIMEOUT_MS){
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

        BitFlags flags = unpackBitFlags(channels[0]);

        switch (flags.id){
        case 0x001:
            if (flags.setEStop) {
                triggerEStop();
                break;
            }

            if (flags.resetEStop && nowMS - EStopTriggerTimeMS > ESTOP_LOCKOUT_MS){
                EStopActive = false;
            }
            
            armed = flags.setArm;

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

    usbRadio.send(DestType::GROUND_STATION, 0x16, payload, 22);
    uartRadio.send(DestType::GROUND_STATION, 0x16, payload, 22);

    //TODO: send imu data
}

void Drone::main()
{
    while(true){
        nowMS = millis();

        if (nowMS - lastSentTelemetry > TELEMETRY_DELAY) {
            sendTelemetry();
        }
        
        usbRadio.update();
        uartRadio.update();

        if (nowMS - lastUSBRecieveTimeMS > TIMEOUT_MS && nowMS - lastUARTRecieveTimeMS > TIMEOUT_MS){
            triggerEStop();
        }

        if (armN.isStalled() || armE.isStalled() || armS.isStalled() || armW.isStalled()){
            triggerEStop();
        }
    }
}

void Drone::triggerEStop(){
    EStopActive = true;
    EStopTriggerTimeMS = nowMS;

    armN.stop();
    armE.stop();
    armS.stop();
    armW.stop();
}