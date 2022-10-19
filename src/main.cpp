#include <Arduino.h>
#include <cannon.h>
#include <ros.h>
#include <std_srvs/SetBool.h>

//#define USE_USBCON

#define SUPPLY_SOLENOID_PIN 2
#define VENT_SOLENOID_PIN 3
#define TANK_0_FILL_SOLENOID_PIN 4
#define TANK_1_FILL_SOLENOID_PIN 5
#define TANK_0_FIRE_SOLENOID_PIN 6
#define TANK_1_FIRE_SOLENOID_PIN 7
#define TANK_0_PRESSURE_SENSOR_PIN A0
#define TANK_1_PRESSURE_SENSOR_PIN A1
#define ANGLE_SENSOR_PIN A2

ros::NodeHandle node_handle;
cannon::cannon *cannon1;
cannon::cannon *cannon2;

cannon::cannon* cannons[2];

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

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> fire_srv("cannon/fire", &fire_cb);



void setup() {
// write your initialization code here
    cannon1 = new cannon::cannon(1, TANK_0_FILL_SOLENOID_PIN,
                                 TANK_0_FIRE_SOLENOID_PIN, TANK_0_PRESSURE_SENSOR_PIN,
                                 "can0/set_pressure", "can0/set_state");
    cannon2 = new cannon::cannon(5, 6, 7, 8,
                                 "can1/set_pressure", "can1/set_state");
    cannons[0] = cannon1;
    cannons[1] = cannon2;
    cannon1->init(&node_handle);
    cannon2->init(&node_handle);

    node_handle.initNode();
    node_handle.advertise(solenoid_pub);
    node_handle.advertise(angle_pub);
    node_handle.advertiseService(fire_srv);

}

void loop() {
// write your code here

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


//    for (auto & cannon : cannons) {
//        cannon->publish_data();
//    }

    // Toggle the on-board LED every loop to indicate that the board is running
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    node_handle.spinOnce();
    // The controllers update rate is 30Hz
    delay(33);
}