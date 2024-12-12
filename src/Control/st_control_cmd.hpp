#pragma once

#include "li_ctrl_mode.hpp"
#include "li_mechanical_frame.hpp"
#include "servo_def.hpp"

struct MFAllJointPosCmd
{
    float ref_joint_position[SERVO_NUM];
};

struct MFAllJointVelCmd
{
    float ref_joint_velocity[SERVO_NUM];
};

struct MFAllJointTrqCmd
{
    float ref_joint_torque[SERVO_NUM];
};
