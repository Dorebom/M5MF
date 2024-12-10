#include "servo_module.hpp"

bool ServoModule::connect_servo() {
    M5_LOGI("ServoModule::connect_servo: ");
    for (int i = 0; i < SERVO_NUM; i++) {
        is_connected = can_servo_.stop_motor(i);
    }
    //
    for (int i = 0; i < SERVO_NUM; i++) {
        can_servo_.get_joint_state(i, servo_state_[i]);
    }
    //
    return is_connected;
}

void ServoModule::get_joint_state(int joint_id, ServoState& state) {
    can_servo_.get_joint_state(joint_id, state);
}

void ServoModule::stop_motor() {
    for (int i = 0; i < SERVO_NUM; i++) {
        can_servo_.stop_motor(i);
    }
}

void ServoModule::enable_motor() {
    for (int i = 0; i < SERVO_NUM; i++) {
        can_servo_.enable_motor(i);
    }
}

ServoModule::ServoModule(/* args */) {
}

ServoModule::~ServoModule() {
}

void ServoModule::init(CommCan* can_driver) {
    can_servo_.set_can_driver(can_driver);

    for (int i = 0; i < SERVO_NUM; i++) {
        can_servo_.register_servo(i, SERVO_INFO.can_id[i],
                                  SERVO_INFO.servo_type[i]);
    }

    for (int i = 0; i < SERVO_NUM; i++) {
        filter_[i].set_param_lpf(1000, 100, 0.7);
    }
}