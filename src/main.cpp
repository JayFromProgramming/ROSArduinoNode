#include <Arduino.h>
#include <cannon.h>
#include <ros.h>

ros::NodeHandle node_handle;
cannon::cannon *cannon1;
cannon::cannon *cannon2;

cannon::cannon* cannons[2];

void setup() {
// write your initialization code here
    cannon1 = new cannon::cannon(1, 2, 3, 4,
                                 "can1/set_pressure", "can1/set_state");
    cannon2 = new cannon::cannon(5, 6, 7, 8,
                                 "can2/set_pressure", "can2/set_state");
    cannons[0] = cannon1;
    cannons[1] = cannon2;
    cannon1->init(&node_handle);
    cannon2->init(&node_handle);
}

void loop() {
// write your code here


    for (auto & cannon : cannons) {
        cannon->publish_data();
    }

    node_handle.spinOnce();
    delay(100);
}