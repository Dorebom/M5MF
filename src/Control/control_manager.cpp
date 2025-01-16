#include "control_manager.hpp"

void ControlManager::get_state() {
    // Timestamp
    local_state.state_code.timestamp = micros();

    ServoState servo_state[SERVO_NUM];
    for (int i = 0; i < SERVO_NUM; i++) {
        servo_.get_joint_state(i, servo_state[i]);
        local_state.act_joint_position[i] = servo_state[i].act_joint_position;
        local_state.act_joint_velocity[i] = servo_state[i].act_joint_velocity;
        local_state.act_joint_torque[i] = servo_state[i].act_joint_torque;
    }
    ForceSensorState temp;
    servo_.get_ext_force_state(temp);

    local_state.act_mf_ext_torque[0] = temp.fx;
    local_state.act_mf_ext_torque[1] = temp.fy;
    local_state.act_mf_ext_torque[2] = temp.mz;
}

void ControlManager::change_data_size() {
    uint8_t data_size = 0;
    switch (local_state.state_code.mf_type) {
        case MECHANICAL_FRAME_LIST::ALLJOINT:
            switch (local_state.state_code.ctrl_mode) {
                case CTRL_MODE_LIST::POSITION:
                    data_size = sizeof(MFAllJointPosState);
                    break;
                case CTRL_MODE_LIST::VELOCITY:
                    data_size = sizeof(MFAllJointVelState);
                    break;
                case CTRL_MODE_LIST::TORQUE:
                    data_size = sizeof(MFAllJointTrqState);
                    break;
                default:
                    break;
            }
            break;
        case MECHANICAL_FRAME_LIST::SCARA:
            switch (local_state.state_code.ctrl_mode) {
                case CTRL_MODE_LIST::POSITION:
                    // TO DO : Implement
                    // data_size = sizeof(MFScaraPosState);
                    break;
                case CTRL_MODE_LIST::VELOCITY:
                    // TO DO : Implement
                    // data_size = sizeof(MFScaraVelState);
                    break;
                case CTRL_MODE_LIST::TORQUE:
                    // TO DO : Implement
                    // data_size = sizeof(MFScaraTrqState);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
    local_state.state_code.data_size = data_size;
}

void ControlManager::check_safety() {
    // Check Safety
    for (int i = 0; i < SERVO_NUM; i++) {
        if (local_state.act_joint_position[i] >
                SERVO_INFO.MAX_THRESHOLD_JOINT_POSITION[i] + 0.01 &&
            local_state.act_joint_position[i] -
                    prev_local_state.act_joint_position[i] >
                0.01) {
            local_state.state_code.is_force_stop = true;
        } else if (local_state.act_joint_position[i] <
                       SERVO_INFO.MIN_THRESHOLD_JOINT_POSITION[i] - 0.01 &&
                   local_state.act_joint_position[i] -
                           prev_local_state.act_joint_position[i] <
                       -0.01) {
            local_state.state_code.is_force_stop = true;
        }
    }
    // NOTE: 速度とトルクは分散が大きいっぽいので、一旦実装から外す
}

bool ControlManager::initialize(CommCan* can_driver) {
    // Initialize Servo Module
    // >> 1. Set CAN Driver
    servo_.init(can_driver);
    // >> 2. Connect Servo
    local_state.state_code.is_connecting_device = servo_.connect_servo();
    if (!local_state.state_code.is_connecting_device) {
        M5_LOGE("ControlManager::initialize: Failed to connect servo");
        return false;
    }

    // Check Data Size
    node_state _state;
    if (_state.check_data_size_over(sizeof(ControlStateCode) +
                                    sizeof(MFAllJointPosState))) {
        M5_LOGE("Data Size Over: ControlState > MAX_NODE_STATE_DATA_SIZE");
        return false;
    }
    if (_state.check_data_size_over(sizeof(ControlStateCode) +
                                    sizeof(MFAllJointVelState))) {
        M5_LOGE("Data Size Over: ControlState > MAX_NODE_STATE_DATA_SIZE");
        return false;
    }
    if (_state.check_data_size_over(sizeof(ControlStateCode) +
                                    sizeof(MFAllJointTrqState))) {
        M5_LOGE("Data Size Over: ControlState > MAX_NODE_STATE_DATA_SIZE");
        return false;
    }
    // Check Control State Data Size
    ControlState _c_state;
    if (_c_state.check_data_size_over(sizeof(MFAllJointPosState))) {
        M5_LOGE(
            "Data Size Over: MFAllJointPosState > MAX_CONTROL_STATE_DATA_SIZE");
        return false;
    }
    if (_c_state.check_data_size_over(sizeof(MFAllJointVelState))) {
        M5_LOGE(
            "Data Size Over: MFAllJointVelState > MAX_CONTROL_STATE_DATA_SIZE");
        return false;
    }
    if (_c_state.check_data_size_over(sizeof(MFAllJointTrqState))) {
        M5_LOGE(
            "Data Size Over: MFAllJointTrqState > MAX_CONTROL_STATE_DATA_SIZE");
        return false;
    }

    return true;
}

void ControlManager::update() {
    //
    if (local_state.state_code.is_connecting_device) {
        // 1. Execute Cmd
        cmd_executor();
        // 2. Get State
        get_state();
        //
        check_safety();

        // 3. Update State and Control Input
        // >> >> operate servo area
        if (local_state.state_code.is_force_stop ||
            !local_state.state_code.is_power_on) {
            servo_.stop_motor();
            local_state.state_code.is_power_on = false;
        } else {
            switch (local_state.state_code.ctrl_mode) {
                case CTRL_MODE_LIST::POSITION:
                    //
                    for (int i = 0; i < SERVO_NUM; i++) {
                        local_state.cmd_joint_position[i] =
                            local_state.ref_joint_position[i];
                    }
                    //
                    if (cmd_watchdog_counter > 10) {
                        for (int i = 0; i < SERVO_NUM; i++) {
                            local_state.cmd_joint_position[i] =
                                local_state.act_joint_position[i];
                        }
                    }
                    //
                    servo_.position_control(local_state);
                    break;
                case CTRL_MODE_LIST::VELOCITY:
                    //
                    for (int i = 0; i < SERVO_NUM; i++) {
                        local_state.cmd_joint_velocity[i] =
                            local_state.ref_joint_velocity[i];
                    }
                    //
                    if (cmd_watchdog_counter > 10) {
                        for (int i = 0; i < SERVO_NUM; i++) {
                            local_state.cmd_joint_velocity[i] = 0.0;
                        }
                    }
                    //
                    servo_.velocity_control(local_state);
                    break;
                case CTRL_MODE_LIST::TORQUE:
                    //
                    for (int i = 0; i < SERVO_NUM; i++) {
                        local_state.cmd_joint_torque[i] =
                            local_state.ref_joint_torque[i];
                    }
                    //
                    if (cmd_watchdog_counter > 5) {
                        for (int i = 0; i < SERVO_NUM; i++) {
                            // local_state.cmd_joint_torque[i] = 0.0;

                            if (std::abs(local_state.act_joint_velocity[i]) >
                                3.0) {
                                local_state.cmd_joint_torque[i] =
                                    -1.0 * local_state.act_joint_velocity[i];
                            } else {
                                local_state.cmd_joint_torque[i] = 0.0;
                            }
                        }
                    }
                    //
                    servo_.torque_control(local_state);
                    break;
                default:
                    break;
            }
        }
        // << << end of operate servo area
        servo_.request_ext_force_state();

        // 4. Send System
        set_control_state2stack();

        // 5. Display
        // M5_LOGD("pos: %f, %f, %f", local_state.act_joint_position[0],
        //        local_state.act_joint_position[1],
        //        local_state.act_joint_position[2]);
        //
    } else {
        local_state.state_code.is_connecting_device = servo_.connect_servo();
        if (!local_state.state_code.is_connecting_device) {
            M5_LOGE("ControlManager::update: Failed to connect servo");
        }
    }
    prev_local_state = local_state;
    return;
}

void ControlManager::cmd_executor() {
    if (cmd_stack_->cmd_stack_.size() != 0) {
        st_node_cmd cmd = cmd_stack_->cmd_stack_.pop();

        MFAllJointPosCmd* mf_aj_cm_p_cmd = (MFAllJointPosCmd*)cmd.data;
        MFAllJointVelCmd* mf_aj_cm_v_cmd = (MFAllJointVelCmd*)cmd.data;
        MFAllJointTrqCmd* mf_aj_cm_t_cmd = (MFAllJointTrqCmd*)cmd.data;

        switch (cmd.cmd_code.cmd_type) {
            case M5MF_CMD_LIST::CS_HALT_OPERATING_SERVO:
                local_state.state_code.is_force_stop = true;
                local_state.state_code.is_power_on = false;
                break;
            case M5MF_CMD_LIST::CS_RAISE_OPERATING_SERVO:
                local_state.state_code.is_force_stop = false;
                break;
            case M5MF_CMD_LIST::CS_POWER_ON:
                if (local_state.state_code.is_force_stop) {
                    M5_LOGE("ControlManager::cmd_executor: Force Stop");
                    break;
                }
                for (int i = 0; i < SERVO_NUM; i++) {
                    local_state.ref_joint_position[i] =
                        local_state.act_joint_position[i];
                    local_state.cmd_joint_position[i] =
                        local_state.act_joint_position[i];
                    local_state.cmd_joint_velocity[i] = 0.0;
                    local_state.cmd_joint_aceleration[i] = 0.0;
                    local_state.cmd_joint_torque[i] = 0.0;
                }
                servo_.enable_motor(local_state);
                local_state.state_code.is_power_on = true;
                break;
            case M5MF_CMD_LIST::CS_POWER_OFF:
                if (local_state.state_code.is_force_stop) {
                    M5_LOGE("ControlManager::cmd_executor: Force Stop");
                    break;
                }
                servo_.stop_motor();
                local_state.state_code.is_power_on = false;
                break;
            case M5MF_CMD_LIST::CS_CHANGE_MF1:
                local_state.state_code.mf_type =
                    MECHANICAL_FRAME_LIST::ALLJOINT;
                break;
            case M5MF_CMD_LIST::CS_CHANGE_MF2:
                local_state.state_code.mf_type = MECHANICAL_FRAME_LIST::SCARA;
                break;
            case M5MF_CMD_LIST::CS_CHANGE_POSITION_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::POSITION;
                if (local_state.state_code.mf_type ==
                    MECHANICAL_FRAME_LIST::ALLJOINT) {
                    local_state.state_code.servo_ctrl_mode =
                        CTRL_MODE_LIST::POSITION;
                }
                servo_.change_ctrl_mode(local_state);
                break;
            case M5MF_CMD_LIST::CS_CHANGE_VELOCITY_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::VELOCITY;
                if (local_state.state_code.mf_type ==
                    MECHANICAL_FRAME_LIST::ALLJOINT) {
                    local_state.state_code.servo_ctrl_mode =
                        CTRL_MODE_LIST::VELOCITY;
                }
                servo_.change_ctrl_mode(local_state);
                break;
            case M5MF_CMD_LIST::CS_CHANGE_TORQUE_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::TORQUE;
                if (local_state.state_code.mf_type ==
                    MECHANICAL_FRAME_LIST::ALLJOINT) {
                    local_state.state_code.servo_ctrl_mode =
                        CTRL_MODE_LIST::TORQUE;
                }
                servo_.change_ctrl_mode(local_state);
                break;
            case M5MF_CMD_LIST::CS_ALLJOINT_POSITION_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::POSITION;
                local_state.state_code.servo_ctrl_mode =
                    CTRL_MODE_LIST::POSITION;
                local_state.state_code.mf_type =
                    MECHANICAL_FRAME_LIST::ALLJOINT;
                if (!local_state.state_code.is_power_on) {
                    break;
                }
                for (int i = 0; i < SERVO_NUM; i++) {
                    local_state.ref_joint_position[i] =
                        mf_aj_cm_p_cmd->ref_joint_position[i];
                }
                cmd_watchdog_counter = 0;
                break;
            case M5MF_CMD_LIST::CS_ALLJOINT_VELOCITY_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::VELOCITY;
                local_state.state_code.servo_ctrl_mode =
                    CTRL_MODE_LIST::VELOCITY;
                local_state.state_code.mf_type =
                    MECHANICAL_FRAME_LIST::ALLJOINT;
                if (!local_state.state_code.is_power_on) {
                    break;
                }
                for (int i = 0; i < SERVO_NUM; i++) {
                    local_state.ref_joint_velocity[i] =
                        mf_aj_cm_v_cmd->ref_joint_velocity[i];
                }
                cmd_watchdog_counter = 0;
                break;
            case M5MF_CMD_LIST::CS_ALLJOINT_TORQUE_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::TORQUE;
                local_state.state_code.servo_ctrl_mode = CTRL_MODE_LIST::TORQUE;
                local_state.state_code.mf_type =
                    MECHANICAL_FRAME_LIST::ALLJOINT;
                if (!local_state.state_code.is_power_on) {
                    break;
                }
                for (int i = 0; i < SERVO_NUM; i++) {
                    local_state.ref_joint_torque[i] =
                        mf_aj_cm_t_cmd->ref_joint_torque[i];
                }
                cmd_watchdog_counter = 0;
                break;
            case M5MF_CMD_LIST::CS_SCARA_POSITION_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::POSITION;
                local_state.state_code.servo_ctrl_mode =
                    CTRL_MODE_LIST::POSITION;
                local_state.state_code.mf_type = MECHANICAL_FRAME_LIST::SCARA;
                break;
            case M5MF_CMD_LIST::CS_SCARA_VELOCITY_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::VELOCITY;
                local_state.state_code.servo_ctrl_mode =
                    CTRL_MODE_LIST::VELOCITY;
                local_state.state_code.mf_type = MECHANICAL_FRAME_LIST::SCARA;
                break;
            case M5MF_CMD_LIST::CS_SCARA_TORQUE_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::TORQUE;
                local_state.state_code.servo_ctrl_mode = CTRL_MODE_LIST::TORQUE;
                local_state.state_code.mf_type = MECHANICAL_FRAME_LIST::SCARA;
                break;
            case M5MF_CMD_LIST::CS_SCARA_POSITRQ_CONTROL:
                local_state.state_code.ctrl_mode = CTRL_MODE_LIST::POSITION;
                local_state.state_code.servo_ctrl_mode = CTRL_MODE_LIST::TORQUE;
                local_state.state_code.mf_type = MECHANICAL_FRAME_LIST::SCARA;
                break;
            default:
                break;
        }
        if (cmd_watchdog_counter < 100) {
            cmd_watchdog_counter++;
        }
    }
    return;
}

std::shared_ptr<NodeStateStack> ControlManager::get_control_state_ptr() {
    return state_stack_;
}

std::shared_ptr<NodeCmdStack> ControlManager::get_control_cmd_ptr() {
    return cmd_stack_;
}

void ControlManager::set_control_state2stack() {
    /*
     * *** CAUTION ***
     * ここで外部システムに送信するデータを設定している
     * したがって、SystemManagerで再度データを圧縮設定する必要はない
     */
    //
    change_data_size();
    //
    st_node_state state;
    ControlState* cs_data = (ControlState*)state.data;
    // Node共通の情報を設定
    state.state_code.node_id = CONTROL_NODE_ID;
    state.state_code.state_machine = node_state_machine::UNCONFIGURED;
    state.state_code.transit_destination_node_state_machine =
        (node_state_machine)CONTROL_NODE_SIGNATURE;
    state.state_code.data_size =
        sizeof(ControlStateCode) + local_state.state_code.data_size;
    // ControlStateの情報を設定
    // >> 1. ControlStateCode
    cs_data->state_code.deepcopy(local_state.state_code);
    // >> 2. ControlStateData
    MFAllJointPosState* mf_aj_cm_p_state = (MFAllJointPosState*)cs_data->data;
    MFAllJointVelState* mf_aj_cm_v_state = (MFAllJointVelState*)cs_data->data;
    MFAllJointTrqState* mf_aj_cm_t_state = (MFAllJointTrqState*)cs_data->data;
    switch (local_state.state_code.mf_type) {
        case MECHANICAL_FRAME_LIST::ALLJOINT:
            switch (local_state.state_code.ctrl_mode) {
                case CTRL_MODE_LIST::POSITION:
                    mf_aj_cm_p_state->deepcopy(local_state);
                    break;
                case CTRL_MODE_LIST::VELOCITY:
                    mf_aj_cm_v_state->deepcopy(local_state);
                    break;
                case CTRL_MODE_LIST::TORQUE:
                    mf_aj_cm_t_state->deepcopy(local_state);
                    break;
                default:
                    break;
            }
            break;
        case MECHANICAL_FRAME_LIST::SCARA:
            switch (local_state.state_code.ctrl_mode) {
                case CTRL_MODE_LIST::POSITION:
                    /* code */
                    break;
                case CTRL_MODE_LIST::VELOCITY:
                    /* code */
                    break;
                case CTRL_MODE_LIST::TORQUE:
                    /* code */
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
    // StateStackにPush
    state_stack_->state_stack_.push(state);
}
