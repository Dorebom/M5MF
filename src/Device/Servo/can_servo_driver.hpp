#pragma once

#include <map>

#include "Control/li_ctrl_mode.hpp"
#include "Device/Comm/can_driver.hpp"
#include "Device/Servo/cybergear_driver.hpp"
#include "Device/Servo/m5roller_driver.hpp"
#include "Device/Servo/st_servo_state.hpp"

enum class CanServoType
{
    M5ROLLER,
    CYBERGEAR,
};

struct ServoTypeInfo
{
    /* data */
    uint8_t can_id;
    CanServoType type;
};

class CanServoDriver {
private:
    /* data */
    CommCan *can_;
    M5RollerDriver m5roller_driver;
    XiaomiCyberGearDriver cybergear_driver;

    std::map<uint8_t, ServoTypeInfo> servo_map_;
    twai_message_t send_msg;
    twai_message_t recv_msg;
    //
    std::map<uint8_t, XiaomiCyberGearStatus> cybergear_status;
    std::map<uint8_t, M5rollerDriverStatus> m5roller_status;

    bool parse(ServoTypeInfo &info, twai_message_t &msg);
    bool callback(ServoTypeInfo &info, twai_message_t &send_packet);

public:
    CanServoDriver(/* args */) {
    }
    ~CanServoDriver() {
    }
    void set_can_driver(CommCan *can_driver) {
        can_ = can_driver;
    }
    bool register_servo(uint8_t joint_id, uint8_t can_id, CanServoType type) {
        if (servo_map_.find(joint_id) != servo_map_.end()) {
            return false;
        } else {
            ServoTypeInfo info;
            info.can_id = can_id;
            info.type = type;
            servo_map_[joint_id] = info;
            if (type == CanServoType::CYBERGEAR) {
                cybergear_status[can_id] = XiaomiCyberGearStatus();
                cybergear_status[can_id].can_id = can_id;
            } else if (type == CanServoType::M5ROLLER) {
                m5roller_status[can_id] = M5rollerDriverStatus();
                m5roller_status[can_id].can_id = can_id;
            }
            return true;
        }
    }
    bool enable_motor(uint8_t joint_id);
    bool stop_motor(uint8_t joint_id);
    bool set_mechanical_position_to_zero(uint8_t joint_id);
    bool position_control(uint8_t joint_id, double target_position);
    bool velocity_control(uint8_t joint_id, double target_velocity);
    bool current_control(uint8_t joint_id, double target_current);
    bool torque_control(uint8_t joint_id, double target_torque);
    void get_joint_state(uint8_t joint_id, ServoState &state);
    void change_ctrl_mode(uint8_t joint_id, CTRL_MODE_LIST ctrl_mode);
};