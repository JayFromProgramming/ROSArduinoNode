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

using rosserial_arduino::Test;

namespace cannon {

    class cannon {
    public:
        enum class cannon_states: uint8_t {
            IDLE, // When the cannon has no pressure and is not waiting for anything
            WAITING, // When the cannon is waiting for the air source to be available
            PRESSURIZING, // When the cannon is pressurizing
            VENTING, // When the cannon is depressurizing
            READY, // When the cannon is ready to fire
            ARMED,  // When the cannon is armed and ready to fire
            FIRING, // When the cannon is firing
        };

        enum class cannon_actions: uint8_t {
            VENT = 0, // Vent the cannon
            SET_AUTO = 1, // Set the cannon to auto mode
            DISABLE_AUTO = 2, // Disable auto mode
            FILL = 3, // Fill the cannon
            ARM = 4, // Arm the cannon
            DISARM = 5, // Disarm the cannon
            FIRE = 6, // Fire the cannon
        };

        void set_pressure_cb(const std_msgs::Float32 &msg);

        void set_action_cb(const std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res);

        void set_state_cb(const std_msgs::UInt8 &msg);

    private:

        uint8_t id{};

        // Declare the ROS node and the publishers and subscriber
        ros::NodeHandle *cannon_node;

        std_msgs::UInt8 state_msg;  // The message to publish the state of the cannon
//        ros::Publisher state_pub = ros::Publisher("state", &state_msg);  // The publisher for the state of the cannon
        ros::Publisher state_pub = ros::Publisher("state", &state_msg);  // The publisher for the state of the cannon

        std_msgs::Float32 pressure_msg;  // The message to publish the pressure of the cannon
        ros::Publisher pressure_pub = ros::Publisher("pressure", &pressure_msg);

        // Declare the state variables
        float pressure;  // The current pressure of the cannon
        float set_pressure;  // The set pressure of the cannon

        bool in_auto = false;  // Whether the cannon is in auto mode or not
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
        }

        void init(ros::NodeHandle *node_ptr);

        void initiate_vent();

        void publish_data();

        void read_pressure();
    };

} // cannon

#endif //ROSARDUINONODE_CANNON_H
