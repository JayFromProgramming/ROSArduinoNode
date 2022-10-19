//
// Created by Jay on 10/16/2022.
//

#include <WString.h>
#include <Arduino.h>
#include "cannon.h"

namespace cannon {

    void cannon::init(ros::NodeHandle *node_ptr) {
        this->cannon_node = node_ptr;
        this->cannon_node->advertise(this->state_pub);
        this->cannon_node->advertise(this->pressure_pub);
        this->cannon_node->subscribe(this->set_pressure_sub_);
        this->cannon_node->subscribe(this->set_state_sub_);
    }

    void cannon::set_pressure_cb(const std_msgs::Float32 &msg) {
        float new_pressure = msg.data;
        if (new_pressure > 0 && new_pressure < 100) {
            this->set_pressure = new_pressure;
        }
    }

    void cannon::initiate_vent(){
        digitalWrite(this->in_solenoid_pin, HIGH);
        this->state = cannon_states::VENTING;
    }

    void cannon::read_pressure(){
        float raw_data = analogRead(this->in_sensor_pin);
        this->pressure = (raw_data / 1024.0f) * 5.0;
    }

    void cannon::publish_data(){
        this->state_msg.data = static_cast<uint8_t>(this->state);
        this->state_pub.publish(&this->state_msg);
        this->pressure_msg.data = this->pressure;
        this->pressure_pub.publish(&this->pressure_msg);
    }


    /**
     * @brief Set the state of the cannon via a ROS message
     * @details This function is called when a message is received on the ROS set_state topic
     *      It sets the state of the cannon to the state specified in the message
     * @note This function is a callback function and should not be called directly
     * @note This method only reacts to one message at a time, it must be set back to CLEAR before it will
     * react to another message
     * @param msg The message containing the state to set the cannon to it is of type std_msgs::UInt8
     */
    void cannon::set_state_cb(const std_msgs::UInt8 &msg) {
        auto action = static_cast<cannon_actions>(msg.data);

        // If the cannon has not been cleared, do not accept another message
        if (action != cannon_actions::CLEAR && !this->input_cleared) {
            return;
        } else this->input_cleared = false;
        switch (action) {
            case cannon_actions::CLEAR:
                this->input_cleared = true;
                break;
            case cannon_actions::VENT:
                this->initiate_vent();
                break;
            case cannon_actions::FILL:
                this->state = cannon_states::PRESSURIZING;
                break;
            case cannon_actions::SET_AUTO:
                this->in_auto = true;
                break;
            case cannon_actions::DISABLE_AUTO:
                this->in_auto = false;
                break;
            case cannon_actions::ARM:
                this->state = cannon_states::ARMED;
                break;
            case cannon_actions::DISARM:
                this->state = cannon_states::IDLE;
                break;
            case cannon_actions::FIRE:
                this->state = cannon_states::FIRING;
                break;
            default:
                this->state = cannon_states::IDLE;
                break;
        }
    }

    bool cannon::is_venting() {
        return false;
    }

    bool cannon::needs_air() {
        if (this->filling || this->in_auto) {
            if (this->filled) {
                if (this->pressure < this->set_pressure - this->pressure_deadband){
                    this->filled = false;
                    return true;
                } else return false;
            } else{
                if (this->pressure > this->set_pressure + this->pressure_deadband){
                    this->filled = true;
                    return false;
                } else return true;
            }
        } else return false;
    }

    bool cannon::is_pressurizing() {
        return this->state == cannon_states::PRESSURIZING;
    }

    void cannon::pressurize() {
        digitalWrite(this->in_solenoid_pin, HIGH);
        this->filling = true;
        this->state = cannon_states::PRESSURIZING;
    }

    void cannon::stop_pressurizing() {
        digitalWrite(this->in_solenoid_pin, LOW);
        this->filling = false;
    }


} // cannon