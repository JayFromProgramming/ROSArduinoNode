//
// Created by Jay on 10/16/2022.
//

#ifndef ROSARDUINONODE_CANNON_H
#define ROSARDUINONODE_CANNON_H

#include <stdint.h>
#include <ros.h>
#include <rosserial_arduino/Test.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <std_srvs/SetBool.h>

#define IDE

#ifdef IDE
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/rosserial_arduino/Test.h"
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_msgs/Float32.h"
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_srvs/SetBool.h"
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_msgs/UInt8.h"
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/ros.h"
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/ros/publisher.h"
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/ros/subscriber.h"
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/ros/service_server.h"
#endif

using rosserial_arduino::Test;


class cannon {
public:
    enum class cannon_states: uint8_t {
        ESTOPPED = 0,  // When the cannon is in an emergency stop state, it will not respond to any commands
        // and cannot be released from this state until power is cycled.
        IDLE = 1, // When the cannon has no pressure and is not waiting for anything
        WAITING_FOR_PRESSURE = 2, // When the cannon is waiting for the air source to be available
        PRESSURIZING = 3, // When the cannon is pressurizing
        VENTING = 4, // When the cannon is depressurizing
        READY = 5, // When the cannon is ready to fire
        ARMED = 6,  // When the cannon is armed and ready to fire
        FIRING = 7, // When the cannon is firing
    };

    enum class cannon_actions: uint8_t {
        CLEAR = 0, // Used to clear the topic and wait for a new message (No new actions will be read until this is set)
        VENT = 1, // Vent the cannon
        SET_AUTO = 2, // Set the cannon to auto mode
        DISABLE_AUTO = 3, // Disable auto mode
        FILL = 4, // Fill the cannon
        ARM = 5, // Arm the cannon
        DISARM = 6, // Disarm the cannon
    };

    void set_pressure_cb(const std_msgs::Float32 &msg);

    void set_state_cb(const std_msgs::UInt8 &msg);

private:

    uint8_t id{};

    std_msgs::UInt8 state_msg;  // The message to publish the state of the cannon
//        ros::Publisher state_pub = ros::Publisher("state", &state_msg);  // The publisher for the state of the cannon
    ros::Publisher state_pub_;  // The publisher for the state of the cannon

    std_msgs::Float32 pressure_msg;  // The message to publish the pressure of the cannon
    ros::Publisher pressure_pub_;

    // Declare tuning parameters
    float max_pressure{1000.0f};  // The maximum pressure the cannon can reach
    float min_pressure{0.0f};  // The minimum pressure the cannon can reach
    float pressure_deadband{0.5f};  // The threshold for the pressure to be considered "stable"

    // Declare the state variables
    float pressure;  // The current pressure of the cannon
    float set_pressure;  // The set pressure of the cannon

    bool input_cleared;  // Whether the input has been cleared and can accept another message
    bool in_auto = false;  // Whether the cannon is in auto mode or not
    bool filled  = false;  // Whether the cannon is filled or not
    uint32_t shot_timer = 0;  // The timer for the shot
    cannon_states state = cannon_states::IDLE;  // The current state of the cannon

    // The subscriber for the pressure to set the cannon to
    ros::Subscriber<std_msgs::Float32, cannon> set_pressure_sub_;

    ros::Subscriber<std_msgs::UInt8, cannon> set_state_sub_;

    uint8_t in_solenoid_pin;
    uint8_t shot_solenoid_pin;
    uint8_t in_sensor_pin;

public:

    cannon(uint8_t id, uint8_t in_solenoid_pin,
           uint8_t shot_solenoid_pin, uint8_t in_sensor_pin,
           const char* pressure_name, const char* state_name, const char* set_pressure_name, const char* set_state_name):
            set_pressure_sub_(pressure_name, &cannon::set_pressure_cb, this),
            set_state_sub_(state_name, &cannon::set_state_cb, this),
            state_pub_(set_state_name, &state_msg),
            pressure_pub_(set_pressure_name, &pressure_msg){
            this->id = id;
            this->in_solenoid_pin = in_solenoid_pin;
            this->shot_solenoid_pin = shot_solenoid_pin;
            this->in_sensor_pin = in_sensor_pin;
            this->pressure = 0;
            this->set_pressure = 45.0;
            this->state = cannon_states::IDLE;
            this->in_auto = false;
            this->input_cleared = true;
            pinMode(in_solenoid_pin, OUTPUT);
            pinMode(shot_solenoid_pin, OUTPUT);
            pinMode(in_sensor_pin, INPUT_PULLUP);
    }

    void init(ros::NodeHandle *node_ptr);

    void read_pressure();

    void publish_state();

    void update();

    cannon_states get_state();

    void set_state(cannon_states new_state);

    /**
     * Called when the cannon needs to release all its pressure
     * @note This is a high priority action and will interrupt any other action of all other cannons
     */
    void initiate_vent();


    /**
     * Indicates if the cannon needs air
     * @returns True if the cannon needs air, false otherwise
     */
    bool needs_air();

    void fire();


};


#endif //ROSARDUINONODE_CANNON_H
