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

void Drone::processIncommingFrame(Radio& source, uint8_t type, uint8_t* payload, uint8_t len){
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
            }

            if (flags.resetEStop && nowMS - EStopTriggerTimeMS > ESTOP_LOCKOUT_MS){
                EStopActive = false;
            }
            
            armed = flags.setArm;
            if (!armed) {
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
        case 0x100:
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

bool Drone::sendTelemetry() {
    
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