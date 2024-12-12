#pragma once

#include "Device/Servo/can_servo_driver.hpp"

#define SERVO_NUM 3
#define POSE_DIM  3

struct ServoInfo
{
    /* data */
    uint8_t can_id[SERVO_NUM];
    CanServoType servo_type[SERVO_NUM];

    double offset_position[SERVO_NUM];

    double MIN_THRESHOLD_JOINT_POSITION[SERVO_NUM];
    double MAX_THRESHOLD_JOINT_POSITION[SERVO_NUM];
    double MIN_THRESHOLD_JOINT_VELOCITY[SERVO_NUM];
    double MAX_THRESHOLD_JOINT_VELOCITY[SERVO_NUM];
    double MIN_THRESHOLD_JOINT_TORQUE[SERVO_NUM];
    double MAX_THRESHOLD_JOINT_TORQUE[SERVO_NUM];

    double THRESHOLD_JOINT_VELOCITY_POS[SERVO_NUM];  // Position Control
                                                     // Modeでの閾値

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

        MIN_THRESHOLD_JOINT_POSITION[0] = -6.283184;
        MIN_THRESHOLD_JOINT_POSITION[1] = -6.283184;
        MIN_THRESHOLD_JOINT_POSITION[2] = -6.283184;

        MAX_THRESHOLD_JOINT_POSITION[0] = 6.283184;
        MAX_THRESHOLD_JOINT_POSITION[1] = 6.283184;
        MAX_THRESHOLD_JOINT_POSITION[2] = 6.283184;

        MIN_THRESHOLD_JOINT_VELOCITY[0] = -10.0;
        MIN_THRESHOLD_JOINT_VELOCITY[1] = -10.0;
        MIN_THRESHOLD_JOINT_VELOCITY[2] = -10.0;

        MAX_THRESHOLD_JOINT_VELOCITY[0] = 10.0;
        MAX_THRESHOLD_JOINT_VELOCITY[1] = 10.0;
        MAX_THRESHOLD_JOINT_VELOCITY[2] = 10.0;

        MIN_THRESHOLD_JOINT_TORQUE[0] = -1.0;
        MIN_THRESHOLD_JOINT_TORQUE[1] = -1.0;
        MIN_THRESHOLD_JOINT_TORQUE[2] = -1.0;

        MAX_THRESHOLD_JOINT_TORQUE[0] = 1.0;
        MAX_THRESHOLD_JOINT_TORQUE[1] = 1.0;
        MAX_THRESHOLD_JOINT_TORQUE[2] = 1.0;

        // POSITION CONTROL MODE
        THRESHOLD_JOINT_VELOCITY_POS[0] = 20.0;
        THRESHOLD_JOINT_VELOCITY_POS[1] = 20.0;
        THRESHOLD_JOINT_VELOCITY_POS[2] = 20.0;
    }
};