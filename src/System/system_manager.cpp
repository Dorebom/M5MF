#include "system_manager.hpp"

SystemManager::SystemManager() {
    // Cmd Stack
    sys_cmd_stack = std::make_shared<NodeCmdStack>(MAX_NODE_CMD_STACK_SIZE);
    sys_cmd_stack->cmd_stack_.set_name("SystemCmdStack");

    // Outer State
    outer_system_state_.state_code.node_id = SYSTEM_NODE_ID;
    outer_system_state_.state_code.data_size = sizeof(SystemState);
    inner_system_state_ = (SystemState*)outer_system_state_.data;
    outer_system_state_.state_code.state_machine =
        node_state_machine::UNCONFIGURED;
    outer_system_state_.state_code.transit_destination_node_state_machine =
        (node_state_machine)SYSTEM_NODE_SIGNATURE;
    // Inner State
    inner_system_state_->init();
    //
    return;
}

SystemManager::~SystemManager() {
    return;
}

bool SystemManager::initialize(int time_interval) {
    return true;
}

bool SystemManager::initialize(int time_interval, CommUdp* udp) {
    udp_ = udp;
    is_init_udp_module = true;

    ext_comm_.initialize(sys_cmd_stack, control_state_stack_,
                         &outer_system_state_, &outer_control_state_,
                         inner_system_state_, udp_);

    heartbeat_.set_system_time_interval(time_interval);

    execute_module_.initialize(&outer_system_state_, inner_system_state_);
    execute_module_.set_cmd_stack(
        ext_comm_.get_cmd_stack_to_external_system_ptr());

    return true;
}

void SystemManager::update() {
    // 1. Set args
    if (force_stop_.check_force_stop()) {
        change_sm_force_stop();
    } else if (force_stop_.check_force_stop_changed()) {
        change_sm_ready();
    }

    if (outer_system_state_.state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        halt_operating_servo();
    } else {
        raise_operating_servo();
    }

    //
    get_control_state();
    //
    check_error_and_alert();

    // 2. Execute Cmd
    cmd_executor();

    // 3. Update State
    is_heartbeat_high = heartbeat_.update();

    switch (outer_system_state_.state_code.state_machine) {
        case node_state_machine::READY:
            if (!inner_system_state_->is_occured_error_ &&
                !inner_system_state_->is_occured_warning_ &&
                !inner_system_state_->is_manual_force_stop) {
                change_sm_stable();
            }
            break;
        case node_state_machine::REPAIR:
            if (!inner_system_state_->is_occured_error_ &&
                !inner_system_state_->is_manual_force_stop) {
                change_sm_stable();
            }
            break;
        default:
            break;
    }

    // 4. Display
    // >> >> >> >> NOTE: Display is not implemented yet << << << <<
    // display_.display();
    display_.display(is_heartbeat_high, &outer_system_state_,
                     inner_system_state_, inner_control_state_);

    // 5. Send External System
    ext_comm_.send();
    return;
}

void SystemManager::change_sm_initilizing() {
    outer_system_state_.state_code.state_machine =
        node_state_machine::INITIALIZING;
}
void SystemManager::change_sm_ready() {
    outer_system_state_.state_code.state_machine = node_state_machine::READY;
}
void SystemManager::change_sm_stable() {
    outer_system_state_.state_code.state_machine = node_state_machine::STABLE;
}
void SystemManager::change_sm_repair() {
    outer_system_state_.state_code.state_machine = node_state_machine::REPAIR;
}
void SystemManager::change_sm_force_stop() {
    outer_system_state_.state_code.state_machine =
        node_state_machine::FORCE_STOP;
}

bool SystemManager::check_force_stop() {
    if (outer_system_state_.state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return true;
    } else {
        return false;
    }
}

void SystemManager::recv_from_external_system() {
    ext_comm_.recv();
}

void SystemManager::cmd_executor() {
    st_node_cmd cmd;

    if (outer_system_state_.state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        // システム内の全ての処理を休眠状態に移行させる
    }

    if (sys_cmd_stack->cmd_stack_.size() != 0) {
        cmd = sys_cmd_stack->cmd_stack_.pop();

        inner_system_state_->act_cmd_type = cmd.cmd_code.cmd_type;
        // M5_LOGI("Act Cmd Type: %d", (int)inner_system_state_->act_cmd_type);
        if ((int)cmd.cmd_code.cmd_type < 100) {
            execute_module_.execute(inner_system_state_->act_cmd_type, &cmd);
        } else {
            transfer_cmd_to_control(&cmd);
        }
    }
}

void SystemManager::transfer_cmd_to_control(st_node_cmd* cmd) {
    if (outer_control_state_.state_code.state_machine ==
        node_state_machine::FORCE_STOP) {
        return;
    }
    ctrl_cmd_stack->cmd_stack_.push(*cmd);
}

void SystemManager::check_error_and_alert() {
    inner_system_state_->is_occured_error_ = false;
    inner_system_state_->is_occured_warning_ = false;

    if (is_init_ctrl_task) {
        if (!inner_control_state_->state_code.is_connecting_device) {
            inner_system_state_->is_occured_error_ = true;
        }
        // if (!inner_control_state_->state_code.is_init_joint_pos) {
        //     inner_system_state_->is_occured_error_ = true;
        // }
    }

    // ERROR
    if (inner_system_state_->is_occured_error_ &&
        (outer_system_state_.state_code.state_machine ==
             node_state_machine::STABLE ||
         outer_system_state_.state_code.state_machine ==
             node_state_machine::REPAIR)) {
        outer_system_state_.state_code.state_machine =
            node_state_machine::FORCE_STOP;
    }
    // ALERT
    if (inner_system_state_->is_occured_warning_ &&
        outer_system_state_.state_code.state_machine ==
            node_state_machine::STABLE) {
        outer_system_state_.state_code.state_machine =
            node_state_machine::REPAIR;
    }
}
