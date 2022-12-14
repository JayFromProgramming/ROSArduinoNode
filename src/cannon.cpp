//
// Created by Jay on 10/16/2022.
//

#include <WString.h>
#include <Arduino.h>
#include "cannon.h"



void cannon::init(ros::NodeHandle *node_ptr) {
    this->nodeHandler = node_ptr;
    node_ptr->advertise(this->state_pub_);
    node_ptr->advertise(this->pressure_pub_);
    node_ptr->advertise(this->auto_pub_);
    node_ptr->subscribe(this->set_pressure_sub_);
    node_ptr->subscribe(this->set_state_sub_);
}

void cannon::set_pressure_cb(const std_msgs::Float32 &msg) {
    float new_pressure = msg.data;
    if (new_pressure > 0 && new_pressure < 100) {
        this->set_pressure = new_pressure;
    }
    String log_msg = "Set pressure to " + String(this->set_pressure);
    this->nodeHandler->loginfo(log_msg.c_str());
}

void cannon::initiate_vent(){
    digitalWrite(this->in_solenoid_pin, LOW);
    this->state = cannon_states::VENTING;
    String msg = "Cannon " + String(this->id) + " is venting";
    this->nodeHandler->loginfo(msg.c_str());
}

void cannon::read_pressure(float pressure) {
    this->pressure = pressure;
}

void cannon::publish_state(){
    this->state_msg.data = static_cast<uint8_t>(this->state);
    this->state_pub_.publish(&this->state_msg);
    this->pressure_msg.data = this->pressure;
    this->pressure_pub_.publish(&this->pressure_msg);
    this->auto_msg.data = static_cast<boolean>(this->in_auto);
    this->auto_pub_.publish(&this->auto_msg);
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
        String log_msg = "Cannon " + String(this->id) + " is not cleared, ignoring message";
        this->nodeHandler->logwarn(log_msg.c_str());
        return;
    }
    if (this->state == cannon_states::ESTOPPED) return; // If the cannon is estopped, do not accept any messages
    switch (action) {
        case cannon_actions::IDLE:
            this->in_auto = false;
            this->set_state(cannon_states::IDLE);
            break;
        case cannon_actions::CLEAR:
            this->input_cleared = true;
            return;
        case cannon_actions::VENT:
            this->set_state(cannon_states::VENTING);
            break;
        case cannon_actions::FILL:
            this->set_state(cannon_states::WAITING_FOR_PRESSURE);
            break;
        case cannon_actions::SET_AUTO:
            this->in_auto = true;
            break;
        case cannon_actions::DISABLE_AUTO:
            this->in_auto = false;
            break;
        case cannon_actions::ARM:
            this->set_state(cannon_states::ARMED);
            break;
        case cannon_actions::DISARM:
        default:
            this->set_state(cannon_states::IDLE);
            break;
    }
    this->input_cleared = false; // Reset the input cleared flag
    String log_msg = "Cannon " + String(this->id) + " received action " + String(static_cast<int>(action));
    this->nodeHandler->loginfo(log_msg.c_str());
}

bool cannon::needs_air() {
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
}


void cannon::update() {

    switch(this->state){
        case cannon_states::ESTOPPED:
        case cannon_states::VENTING:
            if (this->pressure < 5){
                this->set_state(cannon_states::IDLE);
            }
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
                String log_msg = "Cannon " + String(this->id) + " finished firing";
                this->nodeHandler->loginfo(log_msg.c_str());
            }
            break;
        case cannon_states::UNKNOWN:
            break;
    }
}

cannon::cannon_states cannon::get_state() {
    return this->state;
}

void cannon::fire() {
    if (this->state != cannon_states::ARMED) return;
    String log_msg = "Cannon " + String(this->id) + " firing...";
    this->state = cannon_states::FIRING;
    this->shot_timer = millis() + 250;
    digitalWrite(this->shot_solenoid_pin, HIGH);
    this->nodeHandler->loginfo(log_msg.c_str());
}

void cannon::set_state(cannon::cannon_states new_state) {
    if (cannon_states::ESTOPPED == this->state) return;

    // Open the fill valve
    String estop_msg = "Cannon " + String(this->id) + " estopped";
    switch (new_state){
        case cannon_states::ESTOPPED:
            this->nodeHandler->logwarn(estop_msg.c_str());
        case cannon_states::VENTING:
            this->in_auto = false;
            this->initiate_vent();
            break;
        case cannon_states::PRESSURIZING:
            digitalWrite(this->in_solenoid_pin, LOW);
            digitalWrite(this->shot_solenoid_pin, LOW);
            break;
        case cannon_states::WAITING_FOR_PRESSURE:
        case cannon_states::READY:
        case cannon_states::ARMED:
        case cannon_states::IDLE:
            digitalWrite(this->in_solenoid_pin, HIGH);
            digitalWrite(this->shot_solenoid_pin, LOW);
            break;
        default:
            break;
    }

    String log_msg = "Cannon " + String(this->id) + " changed state from " +
            String(static_cast<int>(this->state)) + " to " + String(static_cast<int>(new_state));
    this->state = new_state;
}

void cannon::clear_estop() {
    this->state = cannon_states::IDLE;
    this->set_state(cannon_states::IDLE);

    String log_msg = "Cannon " + String(this->id) + " cleared estop";
    this->nodeHandler->loginfo(log_msg.c_str());
}
