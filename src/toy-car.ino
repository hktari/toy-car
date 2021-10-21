#include "Arduino.h"

enum CarState
{
    OFF,
    RUNNING,
    MOVING
};

const int START_PIN = 0;
const int FRONT_LIGHTS_PINS[] = {1, 2};

CarState car_state = CarState::OFF;

void setup()
{
}

void loop()
{
    /*
    -- START CAR --
        1. The car turns on lights and plays engine rev sfx when held in hand
        2. The car then turns off lights after 5 sec if it hasn't moved or isn't being held anymore

    -- DRIVE CAR --
        1. The car plays dj gas sfx when being pushed forward
        2. The car plays break sfx when turned into a L after having moved forward

    */
    if (digitalRead(START_PIN) == HIGH && car_state == CarState::OFF)
    {
        digitalWrite(FRONT_LIGHTS_PINS[0], HIGH);
        digitalWrite(FRONT_LIGHTS_PINS[1], HIGH);
        
        // TODO: play sfx

        car_state = CarState::RUNNING;
    }
    // else if(car_state == CarState::RUNNING && last_move_time)
}
