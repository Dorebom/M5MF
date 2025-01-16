#include "can_device_driver.hpp"

bool CanDeviceDriver::parse(DeviceTypeInfo& info, twai_message_t& msg) {
    switch (info.type) {
        case CanDeviceType::M5ROLLER:
            // m5roller_driver.update_status(msg, m5roller_status[info.can_id]);
            return true;
            break;
        case CanDeviceType::CYBERGEAR:
            cybergear_driver.update_status(msg, cybergear_status[info.can_id]);
            return true;
            break;
        case CanDeviceType::WACOHFORCE:
            can_force_driver.update_status(msg, force_sensor_status);
            return true;
            break;
        default:
            break;
    }
    return false;
}

bool CanDeviceDriver::callback(DeviceTypeInfo& info,
                               twai_message_t& send_packet) {
    if (can_->callback(send_msg, recv_msg)) {
        return parse(info, recv_msg);  // success:true, fail:false
    }
    return false;
}

bool CanDeviceDriver::enable_motor(uint8_t joint_id) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return false;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                m5roller_driver.enable_motor(info.can_id, send_msg);
                break;
            case CanDeviceType::CYBERGEAR:
                cybergear_driver.enable_motor(info.can_id, send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanDeviceDriver::stop_motor(uint8_t joint_id) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return false;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                m5roller_driver.stop_motor(info.can_id, send_msg);
                break;
            case CanDeviceType::CYBERGEAR:
                cybergear_driver.stop_motor(info.can_id, send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanDeviceDriver::set_mechanical_position_to_zero(uint8_t joint_id) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return false;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                // m5roller_driver.set_mechanical_position_to_zero(info.can_id,
                //                                                 send_msg);
                break;
            case CanDeviceType::CYBERGEAR:
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

bool CanDeviceDriver::position_control(uint8_t joint_id,
                                       double target_position) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return false;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                // m5roller_driver.position_control(info.can_id,
                // target_position,
                //                                  send_msg);
                break;
            case CanDeviceType::CYBERGEAR:
                cybergear_driver.set_position_ref(
                    info.can_id, (float)target_position, send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanDeviceDriver::velocity_control(uint8_t joint_id,
                                       double target_velocity) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return false;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                // m5roller_driver.velocity_control(info.can_id,
                // target_velocity,
                //                                  send_msg);
                break;
            case CanDeviceType::CYBERGEAR:
                cybergear_driver.set_spd_ref(info.can_id,
                                             (float)target_velocity, send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanDeviceDriver::current_control(uint8_t joint_id, double target_current) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return false;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                // m5roller_driver.current_control(info.can_id, target_current,
                //                                 send_msg);
                break;
            case CanDeviceType::CYBERGEAR:
                cybergear_driver.set_iq_ref(info.can_id, (float)target_current,
                                            send_msg);
                break;
            default:
                break;
        }
        return callback(info, send_msg);
    }
    return false;
}

bool CanDeviceDriver::torque_control(uint8_t joint_id, double target_torque) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return false;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                // m5roller_driver.torque_control(info.can_id, target_torque,
                //                                send_msg);
                break;
            case CanDeviceType::CYBERGEAR:
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

void CanDeviceDriver::get_joint_state(uint8_t joint_id, ServoState& state) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];

        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                // m5roller_driver.get_joint_state(info.can_id, state);
                break;
            case CanDeviceType::CYBERGEAR:
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

void CanDeviceDriver::change_ctrl_mode(uint8_t joint_id,
                                       CTRL_MODE_LIST ctrl_mode) {
    if (servo_info_map_.find(joint_id) == servo_info_map_.end()) {
        return;
    } else {
        DeviceTypeInfo info = servo_info_map_[joint_id];
        switch (info.type) {
            case CanDeviceType::M5ROLLER:
                break;
            case CanDeviceType::CYBERGEAR:
                switch (ctrl_mode) {
                    case CTRL_MODE_LIST::POSITION:
                        cybergear_driver.set_limit_spd(info.can_id, 10.0,
                                                       send_msg);
                        callback(info, send_msg);
                        cybergear_driver.set_position_mode(info.can_id,
                                                           send_msg);
                        break;
                    case CTRL_MODE_LIST::VELOCITY:
                        cybergear_driver.set_limit_cur(info.can_id, 6.0,
                                                       send_msg);
                        callback(info, send_msg);
                        cybergear_driver.set_velocity_mode(info.can_id,
                                                           send_msg);
                        break;
                    case CTRL_MODE_LIST::TORQUE:
                        cybergear_driver.set_current_mode(info.can_id,
                                                          send_msg);
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
        callback(info, send_msg);
    }
}

void CanDeviceDriver::get_force_sensor_state(ForceSensorState& state) {
    state = force_sensor_status;
}

bool CanDeviceDriver::request_force_sensor_state(uint8_t can_id) {
    // if (force_sensor_info.can_id != can_id) {
    //     return false;
    // }
    can_force_driver.get_force_state(can_id, send_msg);
    callback(force_sensor_info, send_msg);
    can_force_driver.get_torque_state(can_id, send_msg);
    callback(force_sensor_info, send_msg);

    return true;
}
