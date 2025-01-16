#pragma once

#include <map>

// #include "Device/comm/can_driver.hpp"
#include "Control/Device/can_device_driver.hpp"
// #include "Device/Servo/can_servo_driver.hpp"
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
    // CanServoDriver can_servo_;
    CanDeviceDriver can_device_;
    BiquadFilter filter_[SERVO_NUM];

    // >> Data
    const ServoInfo SERVO_INFO;
    ServoState servo_state_[SERVO_NUM];
    double err_sum_joint_velocity[SERVO_NUM];

    // Flag
    bool is_connected = false;
    CTRL_MODE_LIST prev_servo_ctrl_mode = CTRL_MODE_LIST::STAY;

    // >> Function
public:
    ServoModule(/* args */);
    ~ServoModule();

    void init(CommCan* can_driver);
    bool connect_servo();
    void get_joint_state(int joint_id, ServoState& state);
    void stop_motor();
    void enable_motor(const LocalControlState& state);

    void position_control(const LocalControlState& state);
    void velocity_control(const LocalControlState& state);
    void torque_control(const LocalControlState& state);
    void change_ctrl_mode(const LocalControlState& state);

    void request_ext_force_state();
    void get_ext_force_state(ForceSensorState& state);
};
