#pragma once

#include <memory>

#include "Common/m5mf_def.hpp"
#include "servo_def.hpp"
#include "st_control_state.hpp"

struct ControlStateLimited
{
    MECHANICAL_FRAME_LIST mf_type;
    CTRL_MODE_LIST ctrl_mode;
    //
    unsigned long timestamp;
    bool is_force_stop;
    bool is_connecting_device;
    bool is_power_on;
    bool dummy1;

    float act_joint_position[SERVO_NUM];
    float act_joint_velocity[SERVO_NUM];
    float act_joint_torque[SERVO_NUM];

    float cmd_joint_position[SERVO_NUM];
    float dummy2;

    ControlStateLimited() {
        init();
    }

    void init() {
        mf_type = MECHANICAL_FRAME_LIST::NONE;
        ctrl_mode = CTRL_MODE_LIST::STAY;
        timestamp = 0;
        is_force_stop = false;
        is_connecting_device = false;
        is_power_on = false;
        dummy1 = 0;

        for (int i = 0; i < SERVO_NUM; i++) {
            act_joint_position[i] = 0.0;
            act_joint_velocity[i] = 0.0;
            act_joint_torque[i] = 0.0;
            cmd_joint_position[i] = 0.0;
        }
    }

    void compressed_copy(ControlState& state) {
        mf_type = state.state_code.mf_type;
        ctrl_mode = state.state_code.ctrl_mode;
        timestamp = state.state_code.timestamp;
        is_force_stop = state.state_code.is_force_stop;
        is_connecting_device = state.state_code.is_connecting_device;
        is_power_on = state.state_code.is_power_on;
        dummy1 = 0;

        MF1_State* mf1_state = (MF1_State*)state.data;
        MF2_State* mf2_state = (MF2_State*)state.data;

        switch (mf_type) {
            case MECHANICAL_FRAME_LIST::ALLJOINT:
                for (int i = 0; i < SERVO_NUM; i++) {
                    act_joint_position[i] = mf1_state->act_joint_position[i];
                    act_joint_velocity[i] = mf1_state->act_joint_velocity[i];
                    act_joint_torque[i] = mf1_state->act_joint_torque[i];
                    cmd_joint_position[i] = mf1_state->cmd_joint_value[i];
                }
                break;
            case MECHANICAL_FRAME_LIST::SCARA:
                for (int i = 0; i < SERVO_NUM; i++) {
                    act_joint_position[i] = mf2_state->act_joint_position[i];
                    act_joint_velocity[i] = mf2_state->act_joint_velocity[i];
                    act_joint_torque[i] = mf2_state->act_joint_torque[i];
                    cmd_joint_position[i] = mf2_state->cmd_joint_value[i];
                }
                break;
        }
    }
};