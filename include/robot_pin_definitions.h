#pragma once
#include "Arduino.h"

namespace ackercito_pins
{
    //Actuators
    constexpr uint8_t THROTTLE_DRIVER_PIN = 3;
    constexpr uint8_t STEERING_DRIVER_PIN = 1;

    //2.4GHz RC Receiver
    constexpr uint8_t THROTTLE_RECEIVER_PIN = 6;
    constexpr uint8_t STEERING_RECEIVER_PIN = 5;

    //Battery Sensing
    constexpr uint8_t SBC_BATTERY_SENSE_PIN = A0;
    constexpr uint8_t ESC_BATTERY_SENSE_PIN = A7;
}