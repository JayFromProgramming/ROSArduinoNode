//
// Created by Jay on 10/16/2022.
//

#ifndef ROSARDUINONODE_CANNON_H
#define ROSARDUINONODE_CANNON_H

#include <stdint.h>

namespace cannon {

    class cannon {

        cannon(uint8_t id, uint8_t in_solenoid_pin, uint8_t shot_solenoid_pin, uint8_t in_sensor_pin);

    };

} // cannon

#endif //ROSARDUINONODE_CANNON_H
