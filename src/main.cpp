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
volatile float setpoint_throttle_pulse_ms = ackercito_config::MID_PULSE_THROTTLE; 
volatile float setpoint_steering_pulse_ms = ackercito_config::MID_PULSE_STEERING;
Servo steering;
Servo throttle;
uint32_t last_time_cmd_vel_ms;

//Actuators control loop
IntervalTimer motors_control_timer;



void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
    last_time_cmd_vel_ms = millis();
    nh.loginfo("Received message in cmd_vel");
    float linear_x = cmd_vel_msg.linear.x;
    float angular_z = cmd_vel_msg.angular.z;

    linear_x = constrain(linear_x, ackercito_config::MIN_LINEAR_VELOCITY, ackercito_config::MAX_LINEAR_VELOCITY);
    angular_z = constrain(angular_z, ackercito_config::MIN_ANGULAR_VELOCITY, ackercito_config::MAX_ANGULAR_VELOCITY);

    setpoint_throttle_pulse_ms = map(
        linear_x,
        ackercito_config::MIN_LINEAR_VELOCITY,
        ackercito_config::MAX_LINEAR_VELOCITY,
        -200.0,
        200.0) + ackercito_config::MID_PULSE_THROTTLE;

    setpoint_steering_pulse_ms = map(
        angular_z,
        ackercito_config::MIN_ANGULAR_VELOCITY,
        ackercito_config::MAX_ANGULAR_VELOCITY,
        -200.0,
        200.0) + ackercito_config::MID_PULSE_STEERING;


    //DEBUG.
    char buffer[50];
    snprintf(buffer, sizeof(buffer),"Throttle: %.2f | Steering: %.2f", setpoint_throttle_pulse_ms, setpoint_steering_pulse_ms);
    nh.loginfo(buffer);
    //END DEBUG

    //TODO: STORE THE CURRENT TIME AS THE LAST TIME THAT A COMMAND WAS RECEIVED.
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}


void motors_control_callback()
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    steering.writeMicroseconds(setpoint_steering_pulse_ms);
    throttle.writeMicroseconds(setpoint_throttle_pulse_ms);
    
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
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN,HIGH);
    
    //Block the execution of the program until detecting the ESC (Motors) battery. 
    wait_for_Battery(esc_battery, 4.0); //ToDo: Refactor this 4.0 (Volts).

    //Wait for node connection with ROS
    while (!nh.connected()) {
        nh.spinOnce();
    }
    
    
    nh.subscribe(cmd_vel_sub);

    //Initialization of actuators.
    throttle.attach(ackercito_pins::THROTTLE_DRIVER_PIN);
    steering.attach(ackercito_pins::STEERING_DRIVER_PIN);

    //Start the periodic timer interrupt for motor control.
    motors_control_timer.begin(motors_control_callback, ackercito_config::PERIOD_CONTROL_LOOP_USEC);
}


void loop()
{
    uint32_t current_time_ms = millis();

    //Watchdog for cmd_vel. Reset motor speeds to zero if no new cmd_vel message is received within the specified time limit.
    if ((current_time_ms - last_time_cmd_vel_ms) > ackercito_config::CMD_VEL_WATCHDOG_TIME_MS)
    {
        setpoint_throttle_pulse_ms = ackercito_config::MID_PULSE_THROTTLE; 
        setpoint_steering_pulse_ms = ackercito_config::MID_PULSE_STEERING;
    }
    nh.spinOnce();
}