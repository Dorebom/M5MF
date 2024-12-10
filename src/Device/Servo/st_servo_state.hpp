#pragma once

struct ServoState
{
    double act_joint_position;
    double act_joint_velocity;
    double act_joint_torque;
    double act_temperature;

    void deepcopy(const ServoState& state) {
        act_joint_position = state.act_joint_position;
        act_joint_velocity = state.act_joint_velocity;
        act_joint_torque = state.act_joint_torque;
        act_temperature = state.act_temperature;
    }
};