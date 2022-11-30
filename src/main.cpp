#include <Arduino.h>
#include <cannon.h>
#include <ros.h>
#include <std_srvs/SetBool.h>
#include "../.pio/libdeps/megaatmega2560/Rosserial Arduino Library/src/std_srvs/Empty.h"

//#define USE_USBCON

#define SUPPLY_SOLENOID_PIN 7
#define VENT_SOLENOID_PIN 2
#define TANK_0_FILL_SOLENOID_PIN 6
#define TANK_1_FILL_SOLENOID_PIN 3
#define TANK_0_FIRE_SOLENOID_PIN 8
#define TANK_1_FIRE_SOLENOID_PIN 9
#define TANK_0_PRESSURE_SENSOR_PIN A0
#define TANK_1_PRESSURE_SENSOR_PIN A1
#define ANGLE_SENSOR_PIN A2

//#define SUPPLY_SOLENOID_PIN 2
//#define VENT_SOLENOID_PIN 3
//#define TANK_0_FILL_SOLENOID_PIN 4
//#define TANK_1_FILL_SOLENOID_PIN 5
//#define TANK_0_FIRE_SOLENOID_PIN 6
//#define TANK_1_FIRE_SOLENOID_PIN 7
//#define TANK_0_PRESSURE_SENSOR_PIN A0
//#define TANK_1_PRESSURE_SENSOR_PIN A1
//#define ANGLE_SENSOR_PIN A2


ros::NodeHandle node_handle;
cannon *cannon1;
cannon *cannon2;

cannon* cannons[2];

uint32_t timeouts = 0;

// Create a publisher to publish the solenoid states
std_msgs::UInt8 solenoid_msg;
ros::Publisher solenoid_pub = ros::Publisher("pneumatics/solenoids", &solenoid_msg);
// Create a publisher to publish the cannon angle
std_msgs::Float32 angle_msg;
ros::Publisher angle_pub = ros::Publisher("cannon/angle", &angle_msg);

/**
 * Callback for the cannon Fire service
 * @param req The request message, which is empty
 * @param res The response message, which is empty
 */
void fire_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    for (auto &cannon : cannons) {
        cannon->fire();
    }
}

void clear_estop_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    for (auto &cannon : cannons) {
        cannon->clear_estop();
    }
}

ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> fire_srv("can/fire", &fire_cb);
ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> clear_estop_srv("/can/estop/clear",
                                                                                    &clear_estop_cb);

// rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0

void pinTest(){
    // Test each pin one at a time to determine which pin is causing the crashing problem
    for (int i = 2; i < 10; i++) {
        if (i == 3) continue;
        if (i == 6) continue;
        if (i == 7) continue;

        pinMode(i, OUTPUT);
        digitalWrite(i, HIGH);
        delay(2500);
        digitalWrite(i, LOW);
        delay(1500);
    }

    // Turn all the pings on one at a time but then keep them on
    for (int i = 2; i < 10; i++) {
        if (i == 3) continue;
        if (i == 6) continue;
        if (i == 7) continue;
        pinMode(i, OUTPUT);
        digitalWrite(i, HIGH);

        delay(1000);
    }

    // Turn all the pins off
    for (int i = 2; i < 10; i++) {
        if (i == 3) continue;
        if (i == 6) continue;
        if (i == 7) continue;
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);

        delay(1000);
    }

    for (int i = 2; i < 10; i++) {
        digitalWrite(i, HIGH);
        delay(1000);
    }

}

void setup() {
// write your initialization code here

//    pinTest();

    cannon1 = new cannon(0, TANK_0_FILL_SOLENOID_PIN,
                                 TANK_0_FIRE_SOLENOID_PIN, TANK_0_PRESSURE_SENSOR_PIN,
                                 "can0/set_pressure", "can0/set_state",
                                 "can0/state", "can0/pressure", "can0/auto");
    cannon2 = new cannon(1, TANK_1_FILL_SOLENOID_PIN,
                         TANK_1_FIRE_SOLENOID_PIN,TANK_1_PRESSURE_SENSOR_PIN,
                                 "can1/set_pressure", "can1/set_state",
                                 "can1/state", "can1/pressure", "can1/auto");
    cannons[0] = cannon1;
    cannons[1] = cannon2;
    cannon1->init(&node_handle);
    cannon2->init(&node_handle);

    node_handle.initNode();
    node_handle.advertise(solenoid_pub);
    node_handle.advertise(angle_pub);
    node_handle.advertiseService(fire_srv);
    node_handle.advertiseService(clear_estop_srv);
    node_handle.setSpinTimeout(50);

    pinMode(SUPPLY_SOLENOID_PIN, OUTPUT);
    pinMode(VENT_SOLENOID_PIN, OUTPUT);

    node_handle.requestSyncTime();

}

bool is_supply_in_use(){
    for (auto &cannon : cannons) {
        if (cannon->get_state() == cannon::cannon_states::PRESSURIZING) {
            return true;
        }
    }
    return false;
}

bool check_for_venting(){
    for (auto &cannon : cannons) {
        if (cannon->get_state() == cannon::cannon_states::VENTING ||
            cannon->get_state() == cannon::cannon_states::ESTOPPED) {
            for (auto &other_cannon : cannons) { // Stop all cannons from filling
                if (other_cannon->get_state() == cannon::cannon_states::PRESSURIZING) {
                    other_cannon->set_state(cannon::cannon_states::WAITING_FOR_PRESSURE);
                }
            }
            digitalWrite(SUPPLY_SOLENOID_PIN, LOW); // Close the supply solenoid
            digitalWrite(VENT_SOLENOID_PIN, HIGH); // Open the vent solenoid
            return true; // Return true if we are venting
        }
    }
    return false; // No cannons are venting
}

void cannon_state_loop(){
    for (auto &cannon: cannons){
        cannon->read_pressure();
        cannon->update();
    }
    // Check if any cannons are venting, as this is the highest priority action it is checked first
    if (check_for_venting()){
        return; // If we are venting, we shouldn't delegate the air supply to any cannons
    }
    // Check if any cannons are pressurizing, if so, turn on the supply solenoid
    if (is_supply_in_use()){
        digitalWrite(SUPPLY_SOLENOID_PIN, HIGH);
        digitalWrite(VENT_SOLENOID_PIN, LOW);
    } else {
        digitalWrite(SUPPLY_SOLENOID_PIN, LOW);
        digitalWrite(VENT_SOLENOID_PIN, LOW);
    }
    // Check if any cannons are waiting for pressure, if so, also check if the supply is not in use
    for (auto &cannon : cannons) {
        if (cannon->get_state() == cannon::cannon_states::WAITING_FOR_PRESSURE) {
            if (!is_supply_in_use()) {
                cannon->set_state(cannon::cannon_states::PRESSURIZING);
            }
        }
    }

}


void loop() {
// write your spinCode here
    uint32_t now = millis();
    // Read the solenoid states
    solenoid_msg.data = 0;
    solenoid_msg.data |= digitalRead(SUPPLY_SOLENOID_PIN) << 0;
    solenoid_msg.data |= digitalRead(VENT_SOLENOID_PIN) << 1;
    solenoid_msg.data |= digitalRead(TANK_0_FILL_SOLENOID_PIN) << 2;
    solenoid_msg.data |= digitalRead(TANK_1_FILL_SOLENOID_PIN) << 3;
    solenoid_msg.data |= digitalRead(TANK_0_FIRE_SOLENOID_PIN) << 4;
    solenoid_msg.data |= digitalRead(TANK_1_FIRE_SOLENOID_PIN) << 5;
    solenoid_pub.publish(&solenoid_msg);

    // Read the angle sensor
    angle_msg.data = analogRead(ANGLE_SENSOR_PIN);
    angle_pub.publish(&angle_msg);

    cannon_state_loop();

    for (auto &cannon : cannons) {
        cannon->publish_state();
    }

    // Toggle the on-board LED every loop to indicate that the board is running
//    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));


    int spinCode = node_handle.spinOnce(); // Check ROSCore for new messages (Timeout of 50ms)
    switch(spinCode){
        case ros::SPIN_OK:
            timeouts = 0;
            break;
        case ros::SPIN_TIMEOUT:
            timeouts++;
            if (timeouts > 40) {
                // If we have timed out 20 times in a row, ESTOP the cannons
                for (auto &cannon: cannons) {
                    cannon->set_state(cannon::cannon_states::ESTOPPED);
                }
            }
            break;
        case ros::SPIN_ERR:
            // If we get an error ESTOP the cannons and then reset the board 5 seconds later
            for (auto &cannon : cannons) {
                cannon->set_state(cannon::cannon_states::ESTOPPED);
            }
            break;
        default:
            // Any other spinCode is unknown, so ESTOP the cannons
            for (auto &cannon : cannons) {
                cannon->set_state(cannon::cannon_states::ESTOPPED);
            }
            break;
    }
    // Throttle the update rate to 20Hz
    while (millis() - now < 50) {
        delay(1);
    }
    // Check to see if the send buffer is not empty, if so, send the data
//    if (node_handle.getSendBufferSize() > 0) {
//        node_handle.send();
//    }
}