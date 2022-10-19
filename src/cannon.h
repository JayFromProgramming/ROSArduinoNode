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

namespace cannon {

    class cannon {
    public:
        enum class cannon_states: uint8_t {
            ESTOPPED,  // When the cannon is in an emergency stop state, it will not respond to any commands
            // and cannot be released from this state until power is cycled.
            IDLE, // When the cannon has no pressure and is not waiting for anything
            WAITING, // When the cannon is waiting for the air source to be available
            PRESSURIZING, // When the cannon is pressurizing
            VENTING, // When the cannon is depressurizing
            READY, // When the cannon is ready to fire
            ARMED,  // When the cannon is armed and ready to fire
            FIRING, // When the cannon is firing
        };

        enum class cannon_actions: uint8_t {
            CLEAR = 0, // Used to clear the topic and wait for a new message (No new actions will be read until this is set)
            VENT = 1, // Vent the cannon
            SET_AUTO = 2, // Set the cannon to auto mode
            DISABLE_AUTO = 3, // Disable auto mode
            FILL = 4, // Fill the cannon
            ARM = 5, // Arm the cannon
            DISARM = 6, // Disarm the cannon
            FIRE = 7, // Fire the cannon
        };

        void set_pressure_cb(const std_msgs::Float32 &msg);

        void set_state_cb(const std_msgs::UInt8 &msg);

    private:

        uint8_t id{};

        std_msgs::UInt8 state_msg;  // The message to publish the state of the cannon
//        ros::Publisher state_pub = ros::Publisher("state", &state_msg);  // The publisher for the state of the cannon
        ros::Publisher state_pub = ros::Publisher("state", &state_msg);  // The publisher for the state of the cannon

        std_msgs::Float32 pressure_msg;  // The message to publish the pressure of the cannon
        ros::Publisher pressure_pub = ros::Publisher("pressure", &pressure_msg);

        // Declare tuning parameters
        float max_pressure{1000.0f};  // The maximum pressure the cannon can reach
        float min_pressure{0.0f};  // The minimum pressure the cannon can reach
        float pressure_deadband{0.5f};  // The threshold for the pressure to be considered "stable"

        // Declare the state variables
        float pressure;  // The current pressure of the cannon
        float set_pressure;  // The set pressure of the cannon

        bool input_cleared;  // Whether the input has been cleared and can accept another message
        bool in_auto = false;  // Whether the cannon is in auto mode or not
        bool filling = false;  // Whether the cannon is filling or not
        bool filled  = false;  // Whether the cannon is filled or not
        bool venting = false;  // Whether the cannon is venting or not
        bool armed = false;  // Whether the cannon is armed or not
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
               const char* pressure_name, const char* state_name):
                set_pressure_sub_(pressure_name, &cannon::set_pressure_cb, this),
                set_state_sub_(state_name, &cannon::set_state_cb, this) {
                this->id = id;
                this->in_solenoid_pin = in_solenoid_pin;
                this->shot_solenoid_pin = shot_solenoid_pin;
                this->in_sensor_pin = in_sensor_pin;
                this->pressure = 0;
                this->set_pressure = 45.0;
                this->state = cannon_states::IDLE;
                this->in_auto = false;
                this->input_cleared = true;
        }

        void init(ros::NodeHandle *node_ptr);

        /**
         * Called when the cannon needs to release all its pressure
         * @note This is a high priority action and will interrupt any other action of all other cannons
         */
        void initiate_vent();

        /**
         * @return True if the cannon is venting, false otherwise
         */
        bool is_venting();

        /**
         * Indicates if the cannon needs air
         * @returns True if the cannon needs air, false otherwise
         */
        bool needs_air();

        /**
         * Indicates if the cannon is currently pressurizing
         * @return True if the cannon is pressurizing, false otherwise
         */
        bool is_pressurizing();

        void pressurize();

        void stop_pressurizing();


    };

} // cannon

#endif //ROSARDUINONODE_CANNON_H
