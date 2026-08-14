#include <Arduino.h>
#include "Drone.h"

// Arm pins
static const int armNPWMPin = 20;
static const int armNHallPin = 26;

static const int armEPWMPin = 22; //22
static const int armEHallPin = 21; //21

static const int armSPWMPin = 18;
static const int armSHallPin = 19;

// West and East were flipped on the PCB, because i misplaced the footprints. 
//as such, the pins for east and west on the ECAD are flipped from what is shown in this firmware. 
static const int armWPWMPin = 14; //14
static const int armWHallPin = 15; //15

// Hardware Configuration
static const int PIN_TX = 16;
static const int PIN_RX = 17;
static const int STATUS_LED_PIN = 23;

void setup()
{
    DroneParams params {
        .armNPWMPin = armNPWMPin,
        .armNHallPin = armNHallPin,

        .armEPWMPin = armEPWMPin,
        .armEHallPin = armEHallPin,

        .armSPWMPin = armSPWMPin,
        .armSHallPin = armSHallPin,

        .armWPWMPin = armWPWMPin,
        .armWHallPin = armWHallPin,

        .statusLedPin = STATUS_LED_PIN,

        .serialParam = Serial,
        
        .radioParam = Serial1,
        .txPin = PIN_TX,
        .rxPin = PIN_RX
    };

    Drone drone(params);

    drone.setup();
    drone.main();
}

void loop()
{

}
