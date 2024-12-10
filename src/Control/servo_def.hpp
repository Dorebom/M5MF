#pragma once

#include "Device/Servo/can_servo_driver.hpp"

#define SERVO_NUM 3

struct ServoInfo
{
    /* data */
    uint8_t can_id[SERVO_NUM];
    CanServoType servo_type[SERVO_NUM];

    double offset_position[SERVO_NUM];

    ServoInfo() {
        can_id[0] = 11;
        can_id[1] = 12;
        can_id[2] = 13;
        servo_type[0] = CanServoType::CYBERGEAR;
        servo_type[1] = CanServoType::CYBERGEAR;
        servo_type[2] = CanServoType::CYBERGEAR;

        offset_position[0] = 0.0;
        offset_position[1] = 0.0;
        offset_position[2] = 0.0;
    }
};