#include <Arduino.h>
#include "Drone.h"

// Arm pins
static const int armNPWMPin = 20;
static const int armNHallPin = 26;

static const int armEPWMPin = 14;
static const int armEHallPin = 15;

static const int armSPWMPin = 18;
static const int armSHallPin = 19;

static const int armWPWMPin = 22;
static const int armWHallPin = 21;

// Hardware Configuration
static const int PIN_TX = 16;
static const int PIN_RX = 17;

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