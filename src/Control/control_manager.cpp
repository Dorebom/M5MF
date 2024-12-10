#include "control_manager.hpp"

void ControlManager::get_state() {
    MF1_State* mf1_state = (MF1_State*)state_->data;
    MF2_State* mf2_state = (MF2_State*)state_->data;

    switch (state_->state_code.mf_type) {
        case MECHANICAL_FRAME_LIST::ALLJOINT:
            outer_state_.state_code.data_size =
                sizeof(ControlStateCode) + sizeof(MF1_State);
            break;
        case MECHANICAL_FRAME_LIST::SCARA:
            outer_state_.state_code.data_size =
                sizeof(ControlStateCode) + sizeof(MF2_State);
            break;
        default:
            break;
    }

    ServoState servo_state[SERVO_NUM];
    for (int i = 0; i < SERVO_NUM; i++) {
        servo_.get_joint_state(i, servo_state[i]);
    }
    switch (state_->state_code.mf_type) {
        case MECHANICAL_FRAME_LIST::ALLJOINT:
            for (int i = 0; i < SERVO_NUM; i++) {
                mf1_state->act_joint_position[i] =
                    servo_state[i].act_joint_position;
                mf1_state->act_joint_velocity[i] =
                    servo_state[i].act_joint_velocity;
                mf1_state->act_joint_torque[i] =
                    servo_state[i].act_joint_torque;
            }
            break;
        case MECHANICAL_FRAME_LIST::SCARA:
            for (int i = 0; i < SERVO_NUM; i++) {
                mf2_state->act_joint_position[i] =
                    servo_state[i].act_joint_position;
                mf2_state->act_joint_velocity[i] =
                    servo_state[i].act_joint_velocity;
                mf2_state->act_joint_torque[i] =
                    servo_state[i].act_joint_torque;
            }
            // TODO : Add SCARA State
            break;
        default:
            break;
    }

    // print
    // M5_LOGD("act_joint_position: %f, %f, %f",
    // mf1_state->act_joint_position[0],
    //        mf1_state->act_joint_position[1],
    //        mf1_state->act_joint_position[2]);
}

bool ControlManager::initialize(CommCan* can_driver) {
    // Initialize Servo Module
    // >> Set CAN Driver
    servo_.init(can_driver);

    state_->state_code.is_connecting_device = servo_.connect_servo();
    if (!state_->state_code.is_connecting_device) {
        M5_LOGE("ControlManager::initialize: Failed to connect servo");
        return false;
    }

    return true;
}
void ControlManager::update() {
    //
    if (state_->state_code.is_connecting_device) {
        // 1. Execute Cmd
        cmd_executor();
        // 2. Get State
        get_state();

        // 3. Update State and Control Input

        // >> >> operate servo area
        if (state_->state_code.is_force_stop ||
            !state_->state_code.is_power_on) {
            servo_.stop_motor();
            state_->state_code.is_power_on = false;
        } else {
            // servo_.enable_motor();
        }
        // << << end of operate servo area

        // 4. Send System
        set_control_state2stack();
        //
    } else {
        state_->state_code.is_connecting_device = servo_.connect_servo();
        if (!state_->state_code.is_connecting_device) {
            M5_LOGE("ControlManager::update: Failed to connect servo");
        }
    }
    return;
}

void ControlManager::cmd_executor() {
    if (cmd_stack_->cmd_stack_.size() != 0) {
        st_node_cmd cmd = cmd_stack_->cmd_stack_.pop();

        switch (cmd.cmd_code.cmd_type) {
            case M5MF_CMD_LIST::CS_HALT_OPERATING_SERVO:
                state_->state_code.is_force_stop = true;
                state_->state_code.is_power_on = false;
                break;
            case M5MF_CMD_LIST::CS_RAISE_OPERATING_SERVO:
                state_->state_code.is_force_stop = false;
                break;
            case M5MF_CMD_LIST::CS_POWER_ON:
                if (state_->state_code.is_force_stop) {
                    M5_LOGE("ControlManager::cmd_executor: Force Stop");
                    break;
                }
                servo_.enable_motor();
                state_->state_code.is_power_on = true;
                break;
            case M5MF_CMD_LIST::CS_POWER_OFF:
                if (state_->state_code.is_force_stop) {
                    M5_LOGE("ControlManager::cmd_executor: Force Stop");
                    break;
                }
                servo_.stop_motor();
                state_->state_code.is_power_on = false;
                break;
            case M5MF_CMD_LIST::CS_CHANGE_MF1:
                state_->state_code.mf_type = MECHANICAL_FRAME_LIST::ALLJOINT;
                outer_state_.state_code.data_size =
                    sizeof(ControlStateCode) + sizeof(MF1_State);
                break;
            case M5MF_CMD_LIST::CS_CHANGE_MF2:
                state_->state_code.mf_type = MECHANICAL_FRAME_LIST::SCARA;
                outer_state_.state_code.data_size =
                    sizeof(ControlStateCode) + sizeof(MF2_State);
                break;
            case M5MF_CMD_LIST::CS_CHANGE_POSITION_CONTROL:
                state_->state_code.ctrl_mode = CTRL_MODE_LIST::POSITION;
                break;
            case M5MF_CMD_LIST::CS_CHANGE_VELOCITY_CONTROL:
                state_->state_code.ctrl_mode = CTRL_MODE_LIST::VELOCITY;
                break;
            case M5MF_CMD_LIST::CS_CHANGE_TORQUE_CONTROL:
                state_->state_code.ctrl_mode = CTRL_MODE_LIST::TORQUE;
                break;
            default:
                break;
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
    state_stack_->state_stack_.push(outer_state_);
}
