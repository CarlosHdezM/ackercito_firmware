#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include "robot_pin_definitions.h"
#include "robot_config.h"
#include "simple_battery_reader.h"
#include <Servo.h>

//TEMPORARY (TESTING)
#include <vector>
#include <std_msgs/String.h>
#include <string>


//Globals (ToDo: REFACTOR)
ros::NodeHandle nh;
SimpleBatteryReader esc_battery{ackercito_pins::ESC_BATTERY_SENSE_PIN};
float setpoint_throttle; //DELETE, JUST FOR TESTING
float setpoint_steering;
Servo steering;
Servo throttle;
uint16_t mid_steering_pulse = 1500;
uint16_t mid_throttle_pulse = 1500;


void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
    nh.loginfo("Received message in cmd_vel");
    float linear_x = cmd_vel_msg.linear.x;
    float angular_z = cmd_vel_msg.angular.z;

    linear_x = constrain(linear_x, ackercito_config::MIN_LINEAR_VELOCITY, ackercito_config::MAX_LINEAR_VELOCITY);
    angular_z = constrain(angular_z, ackercito_config::MIN_ANGULAR_VELOCITY, ackercito_config::MAX_ANGULAR_VELOCITY);

    setpoint_throttle = map(
        linear_x,
        ackercito_config::MIN_LINEAR_VELOCITY,
        ackercito_config::MAX_LINEAR_VELOCITY,
        -200.0,
        200.0) + ackercito_config::MID_PULSE_THROTTLE;

    setpoint_steering = map(
        angular_z,
        ackercito_config::MIN_ANGULAR_VELOCITY,
        ackercito_config::MAX_ANGULAR_VELOCITY,
        -200.0,
        200.0) + ackercito_config::MID_PULSE_STEERING;


    //DEBUG.
    char buffer[50];
    snprintf(buffer, sizeof(buffer),"Throttle: %.2f | Steering: %.2f", setpoint_throttle, setpoint_steering);
    nh.loginfo(buffer);
    //END DEBUG

    //TODO: STORE THE CURRENT TIME AS THE LAST TIME THAT A COMMAND WAS RECEIVED.
    steering.writeMicroseconds(setpoint_steering);
    throttle.writeMicroseconds(setpoint_throttle);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}


void wait_for_Battery(SimpleBatteryReader battery, float min_voltage)
{
    nh.loginfo("Waiting for Battery...");
    float batt_volts = 0.0;
    uint32_t prev_time_ms = millis(); //Used for logging at a specified frequency.
    while(batt_volts < min_voltage){
        batt_volts = battery.readVoltage();
        //LOG EVERY 250 ms.
        uint32_t current_time_ms = millis();
        if ( (current_time_ms - prev_time_ms) > 250){
            prev_time_ms = current_time_ms;
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Waiting for battery (Currently: %.2f V).", batt_volts);
            nh.loginfo(buffer);
        }
    }
    delay(50); //Delay for dumb debouncing.
    batt_volts = battery.readVoltage();
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Battery Detected (%.2f V.), proceeding...", batt_volts);
    nh.loginfo(buffer);
}


ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);


void setup()
{
    nh.initNode();
    //Block the execution of the program until detecting the ESC (Motors) battery. 
    wait_for_Battery(esc_battery, 4.0); //Wait until we sense
    nh.subscribe(cmd_vel_sub);
    pinMode(LED_BUILTIN, OUTPUT);

    //Initialization of actuators.
    throttle.attach(ackercito_pins::THROTTLE_DRIVER_PIN);
    steering.attach(ackercito_pins::STEERING_DRIVER_PIN);

    throttle.writeMicroseconds(ackercito_config::MID_PULSE_THROTTLE);
    steering.writeMicroseconds(ackercito_config::MID_PULSE_STEERING);    

}


void loop()
{

    nh.spinOnce();
    //nh.loginfo("Logging info...");
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //delay(1000);
}