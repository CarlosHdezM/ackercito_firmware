#pragma once
#include "Arduino.h"

class SimpleBatteryReader{
    public:
        SimpleBatteryReader(uint8_t reading_pin, float R1 = DEFAULT_R1_, float R2 = DEFAULT_R2_);
        float readVoltage();
        
    private:
        //Class defaults (constants) (shared by all instances of the class).
        static constexpr float DEFAULT_R1_ = 47000.0;
        static constexpr float DEFAULT_R2_ = 10000.0;
        
        //Private member variables.
        uint8_t reading_pin_;
        float R1_;
        float R2_;

};