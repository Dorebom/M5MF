#include "can_servo_driver.hpp"

bool CanServoDriver::parse(ServoTypeInfo& info, twai_message_t& msg) {
    switch (info.type) {
        case CanServoType::M5ROLLER:
            // m5roller_driver.update_status(msg, m5roller_status[info.can_id]);
            return true;
            break;
        case CanServoType::CYBERGEAR:
            cybergear_driver.update_status(msg, cybergear_status[info.can_id]);
            return true;
            break;
        default:
            break;
    }
    return false;
}

bool CanServoDriver::callback(ServoTypeInfo& info,
                              twai_message_t& send_packet) {
    if (can_->callback(send_msg, recv_msg)) {
        return parse(info, recv_msg);  // success:true, fail:false
    }
    return false;
}

bool CanServoDriver::enable_motor(uint8_t joint_id) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return false;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];
        switch (info.type) {
            case CanServoType::M5ROLLER:
                m5roller_driver.enable_motor(info.can_id, send_msg);
                break;
            case CanServoType::CYBERGEAR:
                cybergear_driver.enable_motor(info.can_id, send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanServoDriver::stop_motor(uint8_t joint_id) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return false;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];
        switch (info.type) {
            case CanServoType::M5ROLLER:
                m5roller_driver.stop_motor(info.can_id, send_msg);
                break;
            case CanServoType::CYBERGEAR:
                cybergear_driver.stop_motor(info.can_id, send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanServoDriver::set_mechanical_position_to_zero(uint8_t joint_id) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return false;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];
        switch (info.type) {
            case CanServoType::M5ROLLER:
                // m5roller_driver.set_mechanical_position_to_zero(info.can_id,
                //                                                 send_msg);
                break;
            case CanServoType::CYBERGEAR:
                cybergear_driver.set_mechanical_position_to_zero(info.can_id,
                                                                 send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanServoDriver::position_control(uint8_t joint_id,
                                      double target_position) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return false;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];
        switch (info.type) {
            case CanServoType::M5ROLLER:
                // m5roller_driver.position_control(info.can_id,
                // target_position,
                //                                  send_msg);
                break;
            case CanServoType::CYBERGEAR:
                cybergear_driver.set_position_ref(info.can_id, target_position,
                                                  send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanServoDriver::velocity_control(uint8_t joint_id,
                                      double target_velocity) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return false;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];
        switch (info.type) {
            case CanServoType::M5ROLLER:
                // m5roller_driver.velocity_control(info.can_id,
                // target_velocity,
                //                                  send_msg);
                break;
            case CanServoType::CYBERGEAR:
                cybergear_driver.set_spd_ref(info.can_id, target_velocity,
                                             send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanServoDriver::current_control(uint8_t joint_id, double target_current) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return false;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];
        switch (info.type) {
            case CanServoType::M5ROLLER:
                // m5roller_driver.current_control(info.can_id, target_current,
                //                                 send_msg);
                break;
            case CanServoType::CYBERGEAR:
                cybergear_driver.set_iq_ref(info.can_id, target_current,
                                            send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanServoDriver::torque_control(uint8_t joint_id, double target_torque) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return false;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];
        switch (info.type) {
            case CanServoType::M5ROLLER:
                // m5roller_driver.torque_control(info.can_id, target_torque,
                //                                send_msg);
                break;
            case CanServoType::CYBERGEAR:
                cybergear_driver.set_iq_ref(info.can_id, target_torque / 0.87,
                                            send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

void CanServoDriver::get_joint_state(uint8_t joint_id, ServoState& state) {
    if (servo_map_.find(joint_id) == servo_map_.end()) {
        return;
    } else {
        ServoTypeInfo info = servo_map_[joint_id];

        switch (info.type) {
            case CanServoType::M5ROLLER:
                // m5roller_driver.get_joint_state(info.can_id, state);
                break;
            case CanServoType::CYBERGEAR:
                state.act_joint_position =
                    cybergear_status[info.can_id].act_position;
                state.act_joint_velocity =
                    cybergear_status[info.can_id].act_velocity;
                state.act_joint_torque =
                    cybergear_status[info.can_id].act_effort;
                state.act_temperature =
                    cybergear_status[info.can_id].act_temperature;
                break;
            default:
                break;
        }
    }
}
