#pragma once

#include <map>

// #include "Device/comm/can_driver.hpp"
#include "Device/Servo/can_servo_driver.hpp"
#include "Device/Servo/st_servo_state.hpp"
//
#include "servo_def.hpp"
//
#include "Service/biquad_filter.hpp"
//
#include "Control/st_control_state.hpp"

class ServoModule {
private:
    // std::map<uint8_t, ServoTypeInfo> servo_info_map_;
    CanServoDriver can_servo_;
    BiquadFilter filter_[SERVO_NUM];

    // >> Data
    ServoInfo SERVO_INFO;
    ServoState servo_state_[SERVO_NUM];

    // Flag
    bool is_connected = false;

    // >> Function
public:
    ServoModule(/* args */);
    ~ServoModule();

    void init(CommCan* can_driver);
    bool connect_servo();
    void get_joint_state(int joint_id, ServoState& state);
    void stop_motor();
    void enable_motor();
};
