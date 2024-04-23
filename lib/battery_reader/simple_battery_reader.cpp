#include "simple_battery_reader.h"

SimpleBatteryReader::SimpleBatteryReader(uint8_t reading_pin, float R1, float R2) : 
        reading_pin_(reading_pin), R1_(R1), R2_(R2){
        
}

float SimpleBatteryReader::readVoltage()
{
    uint16_t batt_RAW = analogRead(this->reading_pin_);
    float batt_volts = batt_RAW * (3.3 / 1023.0) * ((R1_ + R2_) / R2_);
    return (abs(batt_volts) > 0.5 ? batt_volts : 0.0);
}