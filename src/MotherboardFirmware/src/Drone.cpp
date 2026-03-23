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
}

void Drone::main()
{
    /*
    Check if time is > than some frequency
    Send telementry
    Check for incoming packets

    Disconnect estop (no packets for 3 sec)
    Stall estop
    */

    while(true){

    }
    return;
}

void Drone::processIncommingFrame(Radio source, uint8_t type, uint8_t* payload, uint8_t len){

}

bool Drone::sendTelemetry() {
    
}