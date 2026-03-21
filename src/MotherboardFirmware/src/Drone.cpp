#include "Drone.h"

/*
ArmController armN;
ArmController armE;
ArmController armS;
ArmController armW;

SerialUSB usbSerial;
SerialUART radio;

void setup();
void main();
*/

static const uint32_t CRSF_BAUD = 460800;

Drone::Drone(DroneParams params) : armN(params.armNPWMPin, params.armNHallPin),
                                   armE(params.armEPWMPin, params.armEHallPin),
                                   armS(params.armSPWMPin, params.armSHallPin),
                                   armW(params.armWPWMPin, params.armWHallPin),
                                   usbSerial(params.serialParam),
                                   radio(params.radioParam)
{
    radio.setTX(params.txPin);
    radio.setRX(params.rxPin);
    radio.begin(CRSF_BAUD);
}

void Drone::setup()
{
    armN.setup();
    armE.setup();
    armS.setup();
    armW.setup();

    usbSerial.begin(115200);
}

void Drone::main()
{
    /*
    Check if time is > than some frequency
    Send telementry
    Check for incoming packets
    Decide between listening to usb or radio 
    Act on incomming packets

    Disconnect estop (no packets for 3 sec)
    Stall estop
    */
}