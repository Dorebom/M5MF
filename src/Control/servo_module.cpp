#include "servo_module.hpp"

bool ServoModule::connect_servo() {
    M5_LOGI("ServoModule::connect_servo: ");
    for (int i = 0; i < SERVO_NUM; i++) {
        is_connected = can_device_.stop_motor(i);
    }
    //
    for (int i = 0; i < SERVO_NUM; i++) {
        can_device_.get_joint_state(i, servo_state_[i]);
    }
    //
    return is_connected;
}

void ServoModule::get_joint_state(int joint_id, ServoState& state) {
    can_device_.get_joint_state(joint_id, state);
}

void ServoModule::stop_motor() {
    for (int i = 0; i < SERVO_NUM; i++) {
        can_device_.stop_motor(i);
    }
}

void ServoModule::enable_motor(const LocalControlState& state) {
    // M5_LOGI("Act Pos: %f, %f, %f", state.act_joint_position[0],
    //         state.act_joint_position[1], state.act_joint_position[2]);

    for (int i = 0; i < SERVO_NUM; i++) {
        can_device_.position_control(i, state.act_joint_position[i]);
        can_device_.velocity_control(i, 0.0);
        can_device_.current_control(i, 0.0);
    }
    for (int i = 0; i < SERVO_NUM; i++) {
        can_device_.enable_motor(i);
    }
}

void ServoModule::position_control(const LocalControlState& state) {
    double cmd;

    for (size_t i = 0; i < SERVO_NUM; i++) {
        cmd = state.cmd_joint_position[i];

        // >> SAFETY
        if (state.act_joint_torque[i] >
            SERVO_INFO.MAX_THRESHOLD_JOINT_TORQUE[i]) {
            cmd = cmd - 0.6 * (state.act_joint_torque[i] -
                               SERVO_INFO.MAX_THRESHOLD_JOINT_TORQUE[i]);
        } else if (state.act_joint_torque[i] <
                   SERVO_INFO.MIN_THRESHOLD_JOINT_TORQUE[i]) {
            cmd = cmd - 0.6 * (state.act_joint_torque[i] -
                               SERVO_INFO.MIN_THRESHOLD_JOINT_TORQUE[i]);
        }

        if (cmd > SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i]) {
            cmd = SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i];
        } else if (cmd < SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i]) {
            cmd = SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i];
        }
        can_device_.position_control(i, cmd);
    }
}

void ServoModule::velocity_control(const LocalControlState& state) {
    double cmd;

    for (size_t i = 0; i < SERVO_NUM; i++) {
        cmd = state.cmd_joint_velocity[i];
        // >> SAFETY
        if (cmd > SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i]) {
            cmd = SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i];
        } else if (cmd < SERVO_INFO.MIN_THRESHOLD_JOINT_VELOCITY[i]) {
            cmd = SERVO_INFO.MIN_THRESHOLD_JOINT_VELOCITY[i];
        }
        //
        if (state.act_joint_position[i] <
                SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i] &&
            cmd < 0.0) {
            cmd = (-1.0) *
                  (state.act_joint_position[i] -
                   SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i]) *
                  20.0;
        } else if (state.act_joint_position[i] >
                       SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i] &&
                   cmd > 0.0) {
            cmd = (-1.0) *
                  (state.act_joint_position[i] -
                   SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i]) *
                  20.0;
        }
        // << SAFETY
        can_device_.velocity_control(i, cmd);
    }
}

void ServoModule::torque_control(const LocalControlState& state) {
    double cmd;
    double v_cmd = 0.0;

    bool is_over_position = false;
    bool is_over_velocity = false;

    const double gain_pos_p = 20.0;
    const double gain_vel_p = 2.0;
    const double gain_vel_i = 0.021;

    double err_trq = 0.0;
    double diff_pos = 0.0;

    for (size_t i = 0; i < SERVO_NUM; i++) {
        is_over_position = false;
        is_over_velocity = false;
        cmd = state.cmd_joint_torque[i];
        // >> SAFETY
        if (state.act_joint_position[i] <
            SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i]) {
            is_over_position = true;
        } else if (state.act_joint_position[i] >
                   SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i]) {
            is_over_position = true;
        }
        if (state.act_joint_velocity[i] <
            SERVO_INFO.MIN_THRESHOLD_JOINT_VELOCITY[i]) {
            is_over_velocity = true;
        } else if (state.act_joint_velocity[i] >
                   SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i]) {
            is_over_velocity = true;
        }
        //
        if (is_over_position) {
            if (state.act_joint_position[i] <
                SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i]) {
                v_cmd =
                    gain_pos_p * (SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i] -
                                  state.act_joint_position[i]);
            } else if (state.act_joint_position[i] >
                       SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i]) {
                v_cmd =
                    gain_pos_p * (SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i] -
                                  state.act_joint_position[i]);
            }

            if (v_cmd < SERVO_INFO.MIN_THRESHOLD_JOINT_VELOCITY[i]) {
                v_cmd = SERVO_INFO.MIN_THRESHOLD_JOINT_VELOCITY[i];
            } else if (v_cmd > SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i]) {
                v_cmd = SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i];
            }
            err_sum_joint_velocity[i] += v_cmd - state.act_joint_velocity[i];

            cmd = gain_vel_p * (v_cmd - state.act_joint_velocity[i]) +
                  gain_vel_i * err_sum_joint_velocity[i];
        } else if (is_over_velocity) {
            if (state.act_joint_velocity[i] <
                SERVO_INFO.MIN_THRESHOLD_JOINT_VELOCITY[i]) {
                err_sum_joint_velocity[i] +=
                    SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i] -
                    state.act_joint_velocity[i];
                cmd = gain_vel_p * (SERVO_INFO.MIN_THRESHOLD_JOINT_VELOCITY[i] -
                                    state.act_joint_velocity[i]) +
                      gain_vel_i * err_sum_joint_velocity[i];
            } else if (state.act_joint_velocity[i] >
                       SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i]) {
                err_sum_joint_velocity[i] +=
                    SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i] -
                    state.act_joint_velocity[i];
                cmd = gain_vel_p * (SERVO_INFO.MAX_THRESHOLD_JOINT_VELOCITY[i] -
                                    state.act_joint_velocity[i]) +
                      gain_vel_i * err_sum_joint_velocity[i];
            }
        } else {
            v_cmd = 0.0;
            for (int j = 0; j < SERVO_NUM; j++) {
                err_sum_joint_velocity[i] = 0.0;
            }
        }
        //
        if (cmd > SERVO_INFO.MAX_THRESHOLD_JOINT_TORQUE[i]) {
            cmd = SERVO_INFO.MAX_THRESHOLD_JOINT_TORQUE[i];
        } else if (cmd < SERVO_INFO.MIN_THRESHOLD_JOINT_TORQUE[i]) {
            cmd = SERVO_INFO.MIN_THRESHOLD_JOINT_TORQUE[i];
        }
        // << SAFETY
        can_device_.torque_control(i, cmd);
    }
}

void ServoModule::change_ctrl_mode(const LocalControlState& state) {
    if (prev_servo_ctrl_mode == state.state_code.servo_ctrl_mode) {
        return;
    }
    //
    switch (state.state_code.servo_ctrl_mode) {
        case CTRL_MODE_LIST::POSITION:
            for (int i = 0; i < SERVO_NUM; i++) {
                can_device_.position_control(i, state.act_joint_position[i]);
                can_device_.change_ctrl_mode(i,
                                             state.state_code.servo_ctrl_mode);
            }
            break;
        case CTRL_MODE_LIST::VELOCITY:
            for (int i = 0; i < SERVO_NUM; i++) {
                can_device_.velocity_control(i, state.act_joint_velocity[i]);
                can_device_.change_ctrl_mode(i,
                                             state.state_code.servo_ctrl_mode);
            }
            break;
        case CTRL_MODE_LIST::TORQUE:
            for (size_t i = 0; i < SERVO_NUM; i++) {
                can_device_.torque_control(i, state.act_joint_torque[i]);
                can_device_.change_ctrl_mode(i,
                                             state.state_code.servo_ctrl_mode);
            }
            break;
        default:
            break;
    }
    //
    prev_servo_ctrl_mode = state.state_code.servo_ctrl_mode;
}

void ServoModule::request_ext_force_state() {
    can_device_.request_force_sensor_state(FORCE_SENSOR_CAN_ID);
}

void ServoModule::get_ext_force_state(ForceSensorState& state) {
    can_device_.get_force_sensor_state(state);
}

ServoModule::ServoModule(/* args */) {
    for (int i = 0; i < SERVO_NUM; i++) {
        err_sum_joint_velocity[i] = 0.0;
    }
}

ServoModule::~ServoModule() {
}

void ServoModule::init(CommCan* can_driver) {
    can_device_.set_can_driver(can_driver);
    // >> Servo
    for (int i = 0; i < SERVO_NUM; i++) {
        can_device_.register_servo(i, SERVO_INFO.can_id[i],
                                   SERVO_INFO.servo_type[i]);
    }

    for (int i = 0; i < SERVO_NUM; i++) {
        filter_[i].set_param_lpf(1000, 100, 0.7);
    }
    // >> Sensor
    can_device_.register_force_sensor(FORCE_SENSOR_CAN_ID);
}