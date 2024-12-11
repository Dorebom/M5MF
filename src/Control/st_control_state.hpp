#pragma once

#include <cstdint>
#include <cstring>

// #include "Common/node_cmd.hpp"
#include "li_ctrl_mode.hpp"
#include "li_mechanical_frame.hpp"
#include "servo_def.hpp"

#define MAX_CONTROL_STATE_DATA_SIZE 150

struct ControlStateCode
{
    //
    MECHANICAL_FRAME_LIST mf_type;
    CTRL_MODE_LIST ctrl_mode;
    //
    unsigned long timestamp;
    bool is_force_stop;
    bool is_connecting_device;
    bool is_power_on;
    uint8_t data_size;  // * 4 byte

    ControlStateCode() {
        init();
    }

    void init() {
        mf_type = MECHANICAL_FRAME_LIST::NONE;
        ctrl_mode = CTRL_MODE_LIST::STAY;
        timestamp = 0;
        is_force_stop = false;
        is_connecting_device = false;
        is_power_on = false;
        data_size = 0;
    }

    void deepcopy(const ControlStateCode& state) {
        mf_type = state.mf_type;
        ctrl_mode = state.ctrl_mode;
        timestamp = state.timestamp;
        is_force_stop = state.is_force_stop;
        is_connecting_device = state.is_connecting_device;
        is_power_on = state.is_power_on;
        data_size = state.data_size;
    }
};

struct LocalControlState
{
    ControlStateCode state_code;
    // 計算で使う、全部の状態量をここに書く
    double act_joint_position[SERVO_NUM];
    double act_joint_velocity[SERVO_NUM];
    double act_joint_aceleration[SERVO_NUM];
    double act_joint_torque[SERVO_NUM];

    double act_mf_position[POSE_DIM];  // Position(2) + Orientation(1)
    double act_mf_velocity[POSE_DIM];
    double act_mf_aceleration[POSE_DIM];
    double act_mf_torque[POSE_DIM];  // Force(2) + Torque(1)

    double act_mf_ext_torque[POSE_DIM];  // Force(2) + Torque(1)

    // MF1(ALLJOINT)の場合
    double cmd_joint_position[SERVO_NUM];
    double cmd_joint_velocity[SERVO_NUM];
    double cmd_joint_aceleration[SERVO_NUM];
    double cmd_joint_torque[SERVO_NUM];
    // MF2(SCARA)の場合
    double cmd_mf_position[POSE_DIM];
    double cmd_mf_velocity[POSE_DIM];
    double cmd_mf_aceleration[POSE_DIM];
    double cmd_mf_torque[POSE_DIM];

    LocalControlState() {
        init();
    }

    void init() {
        state_code.init();
    }

    void deepcopy(const LocalControlState& state) {
        state_code.deepcopy(state.state_code);
        memcpy(act_joint_position, state.act_joint_position,
               sizeof(act_joint_position));
        memcpy(act_joint_velocity, state.act_joint_velocity,
               sizeof(act_joint_velocity));
        memcpy(act_joint_aceleration, state.act_joint_aceleration,
               sizeof(act_joint_aceleration));
        memcpy(act_joint_torque, state.act_joint_torque,
               sizeof(act_joint_torque));
        //
        memcpy(act_mf_position, state.act_mf_position, sizeof(act_mf_position));
        memcpy(act_mf_velocity, state.act_mf_velocity, sizeof(act_mf_velocity));
        memcpy(act_mf_aceleration, state.act_mf_aceleration,
               sizeof(act_mf_aceleration));
        memcpy(act_mf_torque, state.act_mf_torque, sizeof(act_mf_torque));
        memcpy(act_mf_ext_torque, state.act_mf_ext_torque,
               sizeof(act_mf_ext_torque));
        //
        memcpy(cmd_joint_position, state.cmd_joint_position,
               sizeof(cmd_joint_position));
        memcpy(cmd_joint_velocity, state.cmd_joint_velocity,
               sizeof(cmd_joint_velocity));
        memcpy(cmd_joint_aceleration, state.cmd_joint_aceleration,
               sizeof(cmd_joint_aceleration));
        memcpy(cmd_joint_torque, state.cmd_joint_torque,
               sizeof(cmd_joint_torque));
        //
        memcpy(cmd_mf_position, state.cmd_mf_position, sizeof(cmd_mf_position));
        memcpy(cmd_mf_velocity, state.cmd_mf_velocity, sizeof(cmd_mf_velocity));
        memcpy(cmd_mf_aceleration, state.cmd_mf_aceleration,
               sizeof(cmd_mf_aceleration));
        memcpy(cmd_mf_torque, state.cmd_mf_torque, sizeof(cmd_mf_torque));
    }
};

struct ControlState
{
    ControlStateCode state_code;
    std::uint8_t data[MAX_CONTROL_STATE_DATA_SIZE];

    ControlState() {
        init();
    }

    void init() {
        state_code.init();
    }

    void deepcopy(const ControlState& state) {
        state_code.deepcopy(state.state_code);
        memcpy(data, state.data, state_code.data_size);
    }
    bool check_data_size_over(std::uint8_t size) {
        return size > MAX_CONTROL_STATE_DATA_SIZE;
    }
};

struct MF1_State
{
    double act_joint_position[3];
    double act_joint_velocity[3];
    double act_joint_torque[3];

    double cmd_joint_value[3];

    MF1_State() {
        init();
    }

    void init() {
        for (int i = 0; i < 3; i++) {
            act_joint_position[i] = 0.0;
            act_joint_velocity[i] = 0.0;
            act_joint_torque[i] = 0.0;
            cmd_joint_value[i] = 0.0;
        }
    }

    void deepcopy(const MF1_State& state) {
        for (int i = 0; i < 3; i++) {
            act_joint_position[i] = state.act_joint_position[i];
            act_joint_velocity[i] = state.act_joint_velocity[i];
            act_joint_torque[i] = state.act_joint_torque[i];
            cmd_joint_value[i] = state.cmd_joint_value[i];
        }
    }
};

struct MF2_State
{
    double act_joint_position[3];
    double act_joint_velocity[3];
    double act_joint_torque[3];
    double act_mf_position[3];  // 0: x, 1: y, 2: rz
    double act_mf_velocity[3];  // 0: dx, 1: dy, 2: drz
    double act_mf_torque[3];    // 0: fx, 1: fy, 2: mz

    double cmd_mf_value[3];     // 0: x, 1: y, 2: rz
    double cmd_joint_value[3];  // 0: j1, 1: j2, 2: j3

    MF2_State() {
        init();
    }

    void init() {
        for (int i = 0; i < 3; i++) {
            act_joint_position[i] = 0.0;
            act_joint_velocity[i] = 0.0;
            act_joint_torque[i] = 0.0;
            act_mf_position[i] = 0.0;
            act_mf_velocity[i] = 0.0;
            act_mf_torque[i] = 0.0;
            cmd_mf_value[i] = 0.0;
            cmd_joint_value[i] = 0.0;
        }
    }

    void deepcopy(const MF2_State& state) {
        for (int i = 0; i < 3; i++) {
            act_joint_position[i] = state.act_joint_position[i];
            act_joint_velocity[i] = state.act_joint_velocity[i];
            act_joint_torque[i] = state.act_joint_torque[i];
            act_mf_position[i] = state.act_mf_position[i];
            act_mf_velocity[i] = state.act_mf_velocity[i];
            act_mf_torque[i] = state.act_mf_torque[i];
            cmd_mf_value[i] = state.cmd_mf_value[i];
            cmd_joint_value[i] = state.cmd_joint_value[i];
        }
    }
};

struct MFAllJointPosState
{
    float act_joint_position[SERVO_NUM];
    float act_joint_velocity[SERVO_NUM];
    float act_joint_torque[SERVO_NUM];
    //
    float act_mf_position[POSE_DIM];  // Position(2) + Orientation(1)
    float act_mf_velocity[POSE_DIM];
    float act_mf_aceleration[POSE_DIM];
    float act_mf_torque[POSE_DIM];      // Force(2) + Torque(1)
    float act_mf_ext_torque[POSE_DIM];  // Force(2) + Torque(1)
    //
    float cmd_joint_position[SERVO_NUM];
    float cmd_joint_velocity[SERVO_NUM];
    float cmd_joint_acceleraton[SERVO_NUM];

    void deepcopy(const LocalControlState& state) {
        for (int i = 0; i < SERVO_NUM; i++) {
            act_joint_position[i] = state.act_joint_position[i];
            act_joint_velocity[i] = state.act_joint_velocity[i];
            act_joint_torque[i] = state.act_joint_torque[i];
            cmd_joint_position[i] = state.cmd_joint_position[i];
            cmd_joint_velocity[i] = state.cmd_joint_velocity[i];
            cmd_joint_acceleraton[i] = state.cmd_joint_aceleration[i];
        }
        for (int i = 0; i < POSE_DIM; i++) {
            act_mf_position[i] = state.act_mf_position[i];
            act_mf_velocity[i] = state.act_mf_velocity[i];
            act_mf_aceleration[i] = state.act_mf_aceleration[i];
            act_mf_torque[i] = state.act_mf_torque[i];
            act_mf_ext_torque[i] = state.act_mf_ext_torque[i];
        }
    }
};

struct MFAllJointVelState
{
    // TODO: Add Velocity State
};

struct MFAllJointTrqState
{
    // TODO: Add Torque State
};