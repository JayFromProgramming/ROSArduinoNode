//
// Created by Jay on 10/16/2022.
//

#include <WString.h>
#include <Arduino.h>
#include "cannon.h"



void cannon::init(ros::NodeHandle *node_ptr) {
    node_ptr->advertise(this->state_pub_);
    node_ptr->advertise(this->pressure_pub_);
    node_ptr->subscribe(this->set_pressure_sub_);
    node_ptr->subscribe(this->set_state_sub_);
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
    float raw_voltage = analogRead(this->in_sensor_pin);

    // If the voltage is below .5V, the sensor is not connected
    if (raw_voltage < 500) {
        this->set_state(cannon_states::ESTOPPED);
        return;
    }

    this->pressure = (raw_voltage / 1024.0f) * 5.0;
}

void cannon::publish_state(){
    this->state_msg.data = static_cast<uint8_t>(this->state);
    this->state_pub_.publish(&this->state_msg);
    this->pressure_msg.data = this->pressure;
    this->pressure_pub_.publish(&this->pressure_msg);
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
    }
    if (this->state == cannon_states::ESTOPPED) return; // If the cannon is estopped, do not accept any messages
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
        default:
            this->state = cannon_states::IDLE;
            break;
    }
    this->input_cleared = false; // Reset the input cleared flag
}

bool cannon::needs_air() {
    if (this->filled) {
        if (this->pressure < this->set_pressure - this->pressure_deadband){
            this->filled = false;
            return true;
        } else return false;
    } else{
        if (this->pressure > this->set_pressure){
            this->filled = true;
            return false;
        } else return true;
    }
}


void cannon::update() {

    switch(this->state){
        case cannon_states::ESTOPPED:
        case cannon_states::VENTING:
            break;
        case cannon_states::IDLE:
            if (this->in_auto) this->state = cannon_states::WAITING_FOR_PRESSURE;
            break;
        case cannon_states::WAITING_FOR_PRESSURE:
            break;
        case cannon_states::PRESSURIZING:
            if (!this->needs_air()) {
                this->set_state(cannon_states::READY);
            }
            break;
        case cannon_states::READY:
            if (this->needs_air()) this->state = cannon_states::WAITING_FOR_PRESSURE;
            break;
        case cannon_states::ARMED:
            break;
        case cannon_states::FIRING:
            if(this->shot_timer < millis()){
                this->set_state(cannon_states::IDLE);
            }
            break;
    }
}

cannon::cannon_states cannon::get_state() {
    return this->state;
}

void cannon::fire() {
    if (this->state != cannon_states::ARMED) return;
    this->state = cannon_states::FIRING;
    digitalWrite(this->shot_solenoid_pin, HIGH);
}

void cannon::set_state(cannon::cannon_states new_state) {
    if (cannon_states::ESTOPPED == this->state) return;

    // Open the fill valve
    if (cannon_states::PRESSURIZING == new_state) digitalWrite(this->in_solenoid_pin, HIGH);
    if (cannon_states::ARMED == new_state) {
        digitalWrite(this->in_solenoid_pin, LOW);
        digitalWrite(this->shot_solenoid_pin, LOW);
    }


    this->state = new_state;
}
