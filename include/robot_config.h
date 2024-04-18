#pragma once
#include "Arduino.h"

namespace ackercito_config{
    
    //Limits
    constexpr float MAX_LINEAR_VELOCITY = 2.0; //ToDo: Now this is arbitrary. Make sense of this.
    constexpr float MAX_ANGULAR_VELOCITY = 2.0; //ToDo: Now this is arbitrary. Make sense of this.

    constexpr float MIN_LINEAR_VELOCITY = -MAX_LINEAR_VELOCITY; //ToDo: Now this is arbitrary. Make sense of this.
    constexpr float MIN_ANGULAR_VELOCITY = -MAX_ANGULAR_VELOCITY; //ToDo: Now this is arbitrary. Make sense of this.


    //PWM Limits
    constexpr uint16_t MID_PULSE_THROTTLE = 1500;
    constexpr uint16_t MID_PULSE_STEERING = 1500; //ToDo: Tune this to make the robot drive forwards. 
                                                  //Idea: Make it reconfigurable through dynamic_reconfigure and store this constant in the Teensy non-volatile memory (EEPROM)?

}